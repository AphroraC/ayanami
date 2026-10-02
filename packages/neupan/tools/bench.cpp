/*
 * neupan_cpp: C++ port of the NeuPAN planner.
 *
 * Benchmark: replays the dumped reference frames through DUNE and NRMP and
 * reports per-call wall time.
 *
 * Ported from NeuPAN (https://github.com/hanruihua/NeuPAN),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "neupan/dune.hpp"
#include "neupan/nrmp.hpp"
#include "neupan/robot.hpp"
#include "neupan/tensor_io.hpp"
#include "reference_data.hpp"

using namespace neupan;
using Clock = std::chrono::steady_clock;

namespace {

double usSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
}

void stats(const char* name, std::vector<double>& us) {
  std::sort(us.begin(), us.end());
  double sum = 0;
  for (double v : us) sum += v;
  std::printf("%-28s n=%4zu  mean %8.1f us   median %8.1f us   p95 %8.1f us\n",
              name, us.size(), sum / us.size(), us[us.size() / 2],
              us[static_cast<size_t>(us.size() * 0.95)]);
}

}  // namespace

int main() {
  const std::string data = NEUPAN_TEST_DATA_DIR;
  const std::string model = std::string(NEUPAN_MODEL_DIR) + "/diff_default.bin";
  const TensorFile tf = TensorFile::load(data + "/frames.nptf");

  const int T = ref::scalarInt(tf, "meta.T");
  const int n_frames = ref::scalarInt(tf, "meta.n_frames");

  const auto robot = ref::robot(tf);
  const DUNE dune(robot, model);
  NRMP nrmp(robot, ref::nrmpParams(tf));

  struct Call {
    std::vector<Mat2X> pf, op, lam, sp;
    std::vector<Mat22> R;
    std::vector<Mat> mu;
    NomRef nom;
  };
  std::vector<Call> calls;

  for (int f = 0; f < n_frames; ++f) {
    const int n_iters = ref::scalarInt(tf, ref::frameKey(f) + ".n_iters");
    for (int it = 0; it < n_iters; ++it) {
      const std::string pre = ref::iterKey(f, it);
      Call c;
      c.nom = ref::nomRef(tf, pre);
      c.pf = ref::stages<Mat2X>(tf, pre, "pf", T);
      c.R = ref::stages<Mat22>(tf, pre, "R", T);
      c.op = ref::stages<Mat2X>(tf, pre, "op", T);
      c.mu = ref::stages<Mat>(tf, pre, "mu", T);
      c.lam = ref::stages<Mat2X>(tf, pre, "lam", T);
      c.sp = ref::stages<Mat2X>(tf, pre, "sp", T);
      calls.push_back(std::move(c));
    }
  }

  const Eigen::Index pts_per_call = calls[0].pf[0].cols() * (T + 1);
  std::printf("replaying %zu dumped PAM iterations, %ld points per DUNE call\n",
              calls.size(), static_cast<long>(pts_per_call));

  constexpr int kRepeat = 100;
  std::vector<double> dune_us, nrmp_us, frame_us;

  // warm-up (first OSQP init included here, excluded from stats)
  (void)nrmp.solve(calls[0].nom,
                   nrmp.buildFaFb(calls[0].mu, calls[0].lam, calls[0].sp));

  for (int rep = 0; rep < kRepeat; ++rep) {
    for (const Call& c : calls) {
      const auto t0 = Clock::now();
      const DUNE::Output out = dune.forward(c.pf, c.R, c.op);
      dune_us.push_back(usSince(t0));

      const auto t1 = Clock::now();
      const NRMP::Result res = nrmp.solve(
          c.nom,
          nrmp.buildFaFb(out.mu_list, out.lam_list, out.sort_point_list));
      nrmp_us.push_back(usSince(t1));
      frame_us.push_back(usSince(t0));
      if (!res.success) std::printf("warning: solve failed\n");
    }
  }

  stats("DUNE forward (1100 pts)", dune_us);
  stats("NRMP buildFaFb + solve", nrmp_us);
  stats("PAM iteration (DUNE+NRMP)", frame_us);
  std::printf("\nnote: one planner frame = iter_num (2) PAM iterations\n");
  return 0;
}
