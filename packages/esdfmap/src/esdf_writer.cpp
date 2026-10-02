#include "esdfmap/esdf_writer.hpp"

#include <cmath>

#include "esdfmap/edt.hpp"

namespace esdfmap {

EsdfWriter::EsdfWriter(EsdfMap::Ptr map, MaskReader base_reader,
                       EsdfWriterConfig config)
    : map_(std::move(map)),
      base_reader_(std::move(base_reader)),
      config_(config) {}

EsdfWriter::~EsdfWriter() { stop(); }

void EsdfWriter::setSnapshotListener(SnapshotListener listener) {
  snapshot_listener_ = std::move(listener);
}

void EsdfWriter::setOverlaySource(OverlayMaskReader reader,
                TransformLookup lookup) {
  overlay_reader_ = std::move(reader);
  transform_lookup_ = std::move(lookup);
}

void EsdfWriter::start() {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    return;
  }
  thread_ = std::thread([this] { cycleLoop(); });
}

void EsdfWriter::stop() {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) {
    return;
  }
  cycle_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void EsdfWriter::cycleLoop() {
  const auto period =
      config_.update_rate > 0.0
          ? std::chrono::duration<double>(1.0 / config_.update_rate)
          : std::chrono::duration<double>(0.05);
  while (running_.load(std::memory_order_relaxed)) {
    std::unique_lock<std::mutex> lock(cycle_mutex_);
    cycle_cv_.wait_for(lock, period,
                       [this] { return !running_.load(std::memory_order_relaxed); });
    if (!running_.load(std::memory_order_relaxed)) {
      break;
    }
    lock.unlock();
    rebuildOnce();
  }
}

bool EsdfWriter::rebuildOnce() {
  if (!map_ || !base_reader_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(cycle_mutex_);

  EsdfMaskView base;
  if (!base_reader_(base) || base.cells == nullptr || base.width <= 0 ||
      base.height <= 0) {
    return false;
  }
  EsdfMaskView overlay;
  const bool have_overlay =
      overlay_reader_ && overlay_reader_(overlay) && overlay.cells != nullptr &&
      overlay.width > 0 && overlay.height > 0;

  // Generation gate: identical source generations AND unchanged raster
  // geometry mean byte-identical input -- nothing to rebuild. BOTH sources
  // count: the base (static map) rarely changes while the overlay tracks
  // the live marks.
  if (has_last_ && base.generation == last_base_generation_ &&
      (!have_overlay || overlay.generation == last_overlay_generation_) &&
      base.origin_x == last_origin_x_ && base.origin_y == last_origin_y_) {
    skipped_rebuilds_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  last_base_generation_ = base.generation;
  last_overlay_generation_ = have_overlay ? overlay.generation : 0;
  last_origin_x_ = base.origin_x;
  last_origin_y_ = base.origin_y;
  has_last_ = true;

  // ---- fuse: base mask OR projected overlay mask ----
  const std::size_t cells =
      static_cast<std::size_t>(base.width) * static_cast<std::size_t>(base.height);
  fused_.assign(cells, 0);
  for (std::size_t i = 0; i < cells; ++i) {
    fused_[i] = (*base.cells)[i] ? 1 : 0;
  }
  if (have_overlay && transform_lookup_ && overlay.frame != base.frame) {
    // overlay <- base rigid transform: sample each base cell center in the
    // overlay grid (nearest cell; both are cell-aligned geometry, not
    // interpolatable fields).
    double tx = 0.0;
    double ty = 0.0;
    double yaw = 0.0;
    if (transform_lookup_(overlay.frame, base.frame, tx, ty, yaw)) {
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      for (int y = 0; y < base.height; ++y) {
        for (int x = 0; x < base.width; ++x) {
          if (fused_[static_cast<std::size_t>(y) * base.width + x]) {
            continue;  // base geometry wins over the live overlay
          }
          const double wx =
              base.origin_x + (static_cast<double>(x) + 0.5) * base.resolution;
          const double wy =
              base.origin_y + (static_cast<double>(y) + 0.5) * base.resolution;
          const double ox = tx + cos_yaw * wx - sin_yaw * wy;
          const double oy = ty + sin_yaw * wx + cos_yaw * wy;
          const int oxi = static_cast<int>(std::floor(
              (ox - overlay.origin_x) / overlay.resolution - 0.5));
          const int oyi = static_cast<int>(std::floor(
              (oy - overlay.origin_y) / overlay.resolution - 0.5));
          if (oxi < 0 || oyi < 0 || oxi >= overlay.width ||
              oyi >= overlay.height) {
            continue;  // outside the overlay: the base mask stands alone
          }
          if ((*overlay.cells)[static_cast<std::size_t>(oyi) * overlay.width +
                               static_cast<std::size_t>(oxi)]) {
            fused_[static_cast<std::size_t>(y) * base.width + x] = 1;
          }
        }
      }
    }
  }

  // ---- exact signed EDT + publish ----
  if (!signedEdt2d(fused_, base.width, base.height, base.resolution,
                   config_.max_distance, distance_)) {
    return false;
  }
  auto snapshot = std::make_shared<EsdfSnapshot>();
  snapshot->frame = base.frame;
  snapshot->resolution = base.resolution;
  snapshot->size_x = base.width;
  snapshot->size_y = base.height;
  snapshot->origin_x = base.origin_x;
  snapshot->origin_y = base.origin_y;
  snapshot->max_distance = config_.max_distance;
  snapshot->sequence = ++sequence_;
  snapshot->received = std::chrono::steady_clock::now();
  snapshot->distance = distance_;  // copy: the scratch keeps its capacity
  snapshot->occupied = fused_;
  // Keep a const handle for the listener before the publish move.
  const std::shared_ptr<const EsdfSnapshot> published = snapshot;
  map_->publishSnapshot(std::move(snapshot));
  if (snapshot_listener_) {
    snapshot_listener_(published);
  }
  return true;
}

}  // namespace esdfmap
