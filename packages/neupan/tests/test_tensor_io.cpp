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

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "neupan/mlp.hpp"
#include "neupan/tensor_io.hpp"

using namespace neupan;

namespace {

// Byte-level NPTF writer (the format tools/nptf.py writes), so tests can also
// produce broken files.
class Nptf {
 public:
  Nptf& raw(const void* p, size_t n) {
    bytes_.append(static_cast<const char*>(p), n);
    return *this;
  }
  Nptf& u32(uint32_t v) { return raw(&v, sizeof(v)); }
  Nptf& header(uint32_t count, uint32_t version = 1) {
    return raw("NPTF", 4).u32(version).u32(count);
  }
  // Record header plus a row-major payload of `data` (may be short on
  // purpose).
  template <typename T>
  Nptf& record(const std::string& name, uint32_t dtype, uint32_t rows,
               uint32_t cols, const std::vector<T>& data) {
    u32(static_cast<uint32_t>(name.size())).raw(name.data(), name.size());
    u32(dtype).u32(rows).u32(cols);
    return raw(data.data(), data.size() * sizeof(T));
  }
  // Linear layer `L<i>` with weight (rows x cols) filled with `w`.
  Nptf& linear(int i, uint32_t rows, uint32_t cols, float w = 0.1f) {
    const std::string p = "L" + std::to_string(i) + ".linear.";
    record(p + "weight", 0, rows, cols, std::vector<float>(rows * cols, w));
    return record(p + "bias", 0, rows, 1, std::vector<float>(rows, 0.0f));
  }

  std::string write(const std::string& name) const {
    const auto path =
        std::filesystem::temp_directory_path() /
        ("neupan_test_" + std::to_string(::getpid()) + "_" + name);
    std::ofstream(path, std::ios::binary) << bytes_;
    return path.string();
  }

 private:
  std::string bytes_;
};

}  // namespace

TEST(TensorIo, ReadsRowMajorF32AndF64) {
  const std::string path = Nptf()
                               .header(2)
                               .record<float>("a", 0, 2, 3, {1, 2, 3, 4, 5, 6})
                               .record<double>("b", 1, 1, 2, {0.25, -7.5})
                               .write("ok.nptf");
  const TensorFile tf = TensorFile::load(path);

  Mat a(2, 3);
  a << 1, 2, 3, 4, 5, 6;
  EXPECT_EQ(tf.at("a"), a);
  EXPECT_EQ(tf.at("b"), (Mat(1, 2) << 0.25, -7.5).finished());
  EXPECT_EQ(tf.order, (std::vector<std::string>{"a", "b"}));
  EXPECT_TRUE(tf.has("a"));
  EXPECT_THROW(tf.at("missing"), std::runtime_error);
}

TEST(TensorIo, RejectsBrokenFiles) {
  EXPECT_THROW(TensorFile::load("/nonexistent/file.nptf"), std::runtime_error);

  const auto rejects = [](const Nptf& f, const std::string& name) {
    EXPECT_THROW(TensorFile::load(f.write(name)), std::runtime_error) << name;
  };
  rejects(Nptf().raw("NOPE", 4).u32(1).u32(0), "magic");
  rejects(Nptf().header(0, 2), "version");
  rejects(Nptf().header(1).record<float>("x", 7, 1, 1, {1}), "dtype");
  rejects(Nptf().header(1).record<float>("x", 0, 2, 2, {1, 2, 3}), "short");
  rejects(Nptf().header(2).record<float>("x", 0, 1, 1, {1}), "count");
  // Sizes far beyond the file are refused before anything is allocated.
  rejects(Nptf().header(1).record<float>("x", 0, 0xFFFFFFFF, 0xFFFFFFFF, {}),
          "huge");
  rejects(Nptf().header(1).u32(0xFFFFFFFF), "huge_name");
}

TEST(Mlp, LoadsAConsistentStack) {
  const MLP mlp = MLP::load(Nptf()
                                .header(5)
                                .linear(0, 4, 2)
                                .record<float>("L1.relu", 0, 0, 0, {})
                                .linear(2, 3, 4)
                                .write("mlp_ok.nptf"));
  EXPECT_EQ(mlp.inputDim(), 2);
  EXPECT_EQ(mlp.outputDim(), 3);
  EXPECT_EQ(mlp.forward(MatF::Ones(2, 5)).cols(), 5);
}

TEST(Mlp, RejectsMismatchedLayerSizes) {
  // Layer 1 expects 5 inputs but layer 0 produces 4.
  EXPECT_THROW(MLP::load(Nptf().header(4).linear(0, 4, 2).linear(1, 3, 5).write(
                   "mlp_chain.nptf")),
               std::runtime_error);
  // Bias shorter than the weight's output size.
  EXPECT_THROW(
      MLP::load(Nptf()
                    .header(2)
                    .record<float>("L0.linear.weight", 0, 4, 2,
                                   std::vector<float>(8, 0.1f))
                    .record<float>("L0.linear.bias", 0, 3, 1, {0, 0, 0})
                    .write("mlp_bias.nptf")),
      std::runtime_error);
}
