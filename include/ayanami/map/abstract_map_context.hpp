#pragma once

#include <memory>
#include <string>

namespace ayanami::map {

/** Command-port base type: a typed map write channel (e.g. clear-costmap). */
class MapCommandPort {
 public:
  using Ptr = std::shared_ptr<MapCommandPort>;
  virtual ~MapCommandPort() = default;
};

/**
 * @brief Map-family-neutral map context contract.
 *
 * Deliberately minimal: the only family-agnostic facts are the family name
 * and an opaque command port. Everything else (frames, freshness, path
 * validity, read views) is family-specific and lives on the concrete
 * context (e.g. ayanami::costmap::CostmapMapContext). The host owns the
 * concrete context; modules of a family receive the family-typed context
 * at configure time.
 */
class AbstractMapContext {
 public:
  using Ptr = std::shared_ptr<AbstractMapContext>;
  using ConstPtr = std::shared_ptr<const AbstractMapContext>;

  virtual ~AbstractMapContext() = default;

  virtual std::string mapFamily() const = 0;
  virtual MapCommandPort::Ptr commandPort() const { return nullptr; }
};

}  // namespace ayanami::map
