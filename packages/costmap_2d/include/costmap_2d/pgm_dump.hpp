#pragma once

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace costmap_2d {

/// @brief Dump one cost grid as a PGM (P5) plus a <base>.json metadata
/// sidecar, so a dump can be viewed directly AND re-loaded through the
/// framework's own static-layer path (resolution/origin travel in the
/// sidecar, the same schema the static layer's map json uses).
///
/// Pixel mapping is diagnostic-oriented: pixel = 255 - cost, so FREE_SPACE
/// renders white, the inflation band darkens as a gradient, and lethal
/// cells render black. NOTE: this is a cost visualization, not a
/// map_server round-trip -- reloading the PGM through the trinary static
/// semantics turns the inner inflation band into walls.
///
/// PGM rows are top-down while map row 0 is world -Y: image row i carries
/// map row (size_y - 1 - i), mirroring the static layer's loader flip.
///
/// Atomic-ish write: data lands at <base>.pgm.tmp / <base>.json.tmp and is
/// renamed into place, so an external viewer never sees a torn file.
///
/// @return false with *error set on any I/O failure.
inline bool writePgmDump(const unsigned char* costs, unsigned int size_x,
                         unsigned int size_y, double resolution,
                         double origin_x, double origin_y,
                         const std::filesystem::path& base,
                         std::string* error) {
  auto fail = [error](const std::string& what) {
    if (error) {
      *error = what;
    }
    return false;
  };
  if (size_x == 0 || size_y == 0 || costs == nullptr) {
    return fail("empty cost grid");
  }

  std::error_code ec;
  std::filesystem::create_directories(base.parent_path(), ec);

  // Row-flipped pixel buffer: image row i = map row (size_y - 1 - i).
  std::vector<unsigned char> pixels(static_cast<std::size_t>(size_x) * size_y);
  for (unsigned int y = 0; y < size_y; ++y) {
    const unsigned char* src = costs + static_cast<std::size_t>(y) * size_x;
    unsigned char* dst =
        pixels.data() +
        static_cast<std::size_t>(size_y - 1 - y) * size_x;
    for (unsigned int x = 0; x < size_x; ++x) {
      dst[x] = static_cast<unsigned char>(255u - src[x]);
    }
  }

  const auto pgm_path = std::filesystem::path(base.string() + ".pgm");
  {
    std::ofstream out(pgm_path.string() + ".tmp", std::ios::binary);
    if (!out) {
      return fail("cannot open " + pgm_path.string() + ".tmp");
    }
    out << "P5\n# ayanami costmap dump (pixel = 255 - cost)\n"
        << size_x << " " << size_y << "\n255\n";
    out.write(reinterpret_cast<const char*>(pixels.data()),
              static_cast<std::streamsize>(pixels.size()));
    out.close();
    if (!out) {
      return fail("failed writing " + pgm_path.string());
    }
  }

  // Metadata sidecar: the static layer's map-json schema (resolution +
  // origin), so the geometry travels with the pixels.
  {
    std::ofstream meta(base.string() + ".json.tmp");
    if (!meta) {
      return fail("cannot open " + base.string() + ".json.tmp");
    }
    meta << "{\"map\": {\"resolution\": " << resolution
         << ", \"origin\": [" << origin_x << ", " << origin_y << ", 0.0]}}";
    meta.close();
    if (!meta) {
      return fail("failed writing " + base.string() + ".json");
    }
  }

  std::filesystem::rename(pgm_path.string() + ".tmp", pgm_path, ec);
  if (ec) {
    return fail("rename failed: " + ec.message());
  }
  std::filesystem::rename(base.string() + ".json.tmp",
                          base.string() + ".json", ec);
  if (ec) {
    return fail("rename failed: " + ec.message());
  }
  return true;
}

}  // namespace costmap_2d
