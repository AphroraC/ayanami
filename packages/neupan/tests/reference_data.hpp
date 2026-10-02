/*
 * neupan_cpp: C++ port of the NeuPAN planner.
 *
 * Ported from NeuPAN (https://github.com/hanruihua/NeuPAN),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

// Readers for the reference data that tools/dump_test_data.py records from
// the Python implementation, shared by the tests and tools/bench.cpp.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "neupan/nrmp.hpp"
#include "neupan/robot.hpp"
#include "neupan/tensor_io.hpp"
#include "neupan/types.hpp"

namespace neupan::ref {

inline std::string pad2(int v) {
  return (v >= 0 && v < 10 ? "0" : "") + std::to_string(v);
}

// Record-name prefixes: frame f, PAM iteration `it` of frame f, stage t, and
// replay frame f.
inline std::string frameKey(int f) { return "f" + pad2(f); }
inline std::string iterKey(int f, int it) {
  return frameKey(f) + "_it" + std::to_string(it);
}
inline std::string stageKey(int t) { return "_s" + pad2(t); }
inline std::string replayKey(int f) { return "r" + pad2(f); }

inline int scalarInt(const TensorFile& tf, const std::string& name) {
  return static_cast<int>(tf.at(name)(0, 0));
}

// The diff robot of the recorded scenario (example/LON/planner.yaml):
// max_speed [8, 1], max_acce [8, 3], length 1.6, width 2.0.
inline std::shared_ptr<const Robot> robot(const TensorFile& tf) {
  return std::make_shared<const Robot>(
      Robot::diffRectangle(scalarInt(tf, "meta.T"), tf.at("meta.dt")(0, 0),
                           Vec2(8, 1), Vec2(8, 3), 1.6, 2.0));
}

inline NRMPParams nrmpParams(const TensorFile& tf) {
  NRMPParams p;
  p.max_num = scalarInt(tf, "meta.max_num");
  p.q_s = tf.at("meta.q_s").col(0);
  p.p_u = tf.at("meta.p_u")(0, 0);
  p.eta = tf.at("meta.eta")(0, 0);
  p.d_max = tf.at("meta.d_max")(0, 0);
  p.d_min = tf.at("meta.d_min")(0, 0);
  p.ro_obs = tf.at("meta.ro_obs")(0, 0);
  p.bk = tf.at("meta.bk")(0, 0);
  return p;
}

// NRMP inputs of one recorded PAM iteration (`pre` from iterKey).
inline NomRef nomRef(const TensorFile& tf, const std::string& pre) {
  return NomRef{tf.at(pre + ".nom_s"), tf.at(pre + ".nom_u"),
                tf.at(pre + ".ref_s"), tf.at(pre + ".ref_us").col(0)};
}

// Per-stage records `<pre>.<field>_sXX` for stages 0..T.
template <typename M>
std::vector<M> stages(const TensorFile& tf, const std::string& pre,
                      const std::string& field, int T) {
  std::vector<M> out;
  out.reserve(T + 1);
  for (int t = 0; t <= T; ++t)
    out.emplace_back(tf.at(pre + "." + field + stageKey(t)));
  return out;
}

}  // namespace neupan::ref
