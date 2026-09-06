# fast_tf (vendored)

Vendored from https://github.com/dorezyuk/fast_tf (MIT License).

This copy is a structural fork (API moved into `fast_tf::detail`, tests/perf/
ROS layer stripped) with local modifications for ayanami:

- Configurable dynamic-link cache window: `transform_buffer(duration_t)`
  (upstream hardcodes 500ms); propagated to every tree so links created in
  the pre-created initial tree inherit the configured window.
- `dynamic_transform::latest_time()` — newest sample stamp per link.
- `transform_buffer::latest_common_time()` — tf2-style latest common time
  (minimum of the newest sample stamps along the common ancestor path);
  multi-hop `latest` (zero) queries resolve to it for a consistent snapshot.

See the upstream repository for the full MIT license text.
