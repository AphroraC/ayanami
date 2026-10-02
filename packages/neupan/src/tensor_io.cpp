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

#include "neupan/tensor_io.hpp"

#include <cstdint>
#include <fstream>
#include <stdexcept>

namespace neupan {

namespace {

constexpr uint32_t kDtypeF32 = 0;
constexpr uint32_t kDtypeF64 = 1;

class Reader {
 public:
  explicit Reader(const std::string& path)
      : path_(path), f_(path, std::ios::binary | std::ios::ate) {
    if (!f_) throw std::runtime_error("tensor_io: cannot open " + path);
    remaining_ = static_cast<uint64_t>(f_.tellg());
    f_.seekg(0);
  }

  // Refuses reads past the end of the file before allocating for them, so a
  // corrupt size field cannot trigger a huge allocation.
  void read(void* dst, uint64_t bytes) {
    if (bytes > remaining_) fail("truncated data");
    f_.read(static_cast<char*>(dst), static_cast<std::streamsize>(bytes));
    if (!f_) fail("truncated data");
    remaining_ -= bytes;
  }

  uint32_t u32() {
    uint32_t v;
    read(&v, sizeof(v));
    return v;
  }

  std::string str(uint32_t len) {
    if (len > remaining_) fail("truncated data");
    std::string s(len, '\0');
    read(s.data(), len);
    return s;
  }

  // Row-major payload of `rows` x `cols` scalars of type T.
  template <typename T>
  Mat matrix(uint32_t rows, uint32_t cols) {
    const uint64_t bytes = uint64_t{rows} * cols * sizeof(T);
    if (bytes > remaining_) fail("truncated data");
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> m(rows,
                                                                        cols);
    read(m.data(), bytes);
    return m.template cast<double>();
  }

  [[noreturn]] void fail(const std::string& what) const {
    throw std::runtime_error("tensor_io: " + what + " in " + path_);
  }

 private:
  std::string path_;
  std::ifstream f_;
  uint64_t remaining_ = 0;
};

}  // namespace

TensorFile TensorFile::load(const std::string& path) {
  Reader in(path);

  char magic[4];
  in.read(magic, sizeof(magic));
  if (std::string(magic, 4) != "NPTF") in.fail("bad magic");
  if (in.u32() != 1) in.fail("unsupported version");

  TensorFile out;
  const uint32_t count = in.u32();
  for (uint32_t i = 0; i < count; ++i) {
    const std::string name = in.str(in.u32());

    const uint32_t dtype = in.u32();
    const uint32_t rows = in.u32();
    const uint32_t cols = in.u32();

    Mat m;
    if (dtype == kDtypeF32)
      m = in.matrix<float>(rows, cols);
    else if (dtype == kDtypeF64)
      m = in.matrix<double>(rows, cols);
    else
      in.fail("unknown dtype");

    out.tensors[name] = std::move(m);
    out.order.push_back(name);
  }
  return out;
}

const Mat& TensorFile::at(const std::string& name) const {
  auto it = tensors.find(name);
  if (it == tensors.end())
    throw std::runtime_error("tensor_io: missing tensor " + name);
  return it->second;
}

}  // namespace neupan
