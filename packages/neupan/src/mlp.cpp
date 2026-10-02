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

#include "neupan/mlp.hpp"

#include <stdexcept>
#include <string>

#include "neupan/tensor_io.hpp"

namespace neupan {

MLP MLP::load(const std::string& path) {
  return fromTensors(TensorFile::load(path));
}

MLP MLP::fromTensors(const TensorFile& tf) {
  MLP mlp;
  Layer cur{};
  bool pending = false;
  std::string pendingPrefix;

  auto flush = [&]() {
    if (pending) {
      mlp.layers.push_back(cur);
      cur = Layer{};
      pending = false;
    }
  };

  for (const auto& name : tf.order) {
    const auto dot = name.find('.');
    if (dot == std::string::npos)
      throw std::runtime_error("mlp: bad record name " + name);
    const std::string prefix = name.substr(0, dot);  // "L<i>"
    const std::string rest = name.substr(dot + 1);   // e.g. "linear.weight"

    // Non-layer records (e.g. the trained footprint) belong to other readers.
    if (prefix == "meta") continue;

    if (pending && prefix != pendingPrefix) flush();
    pendingPrefix = prefix;

    const Mat& m = tf.at(name);
    if (rest == "linear.weight") {
      cur.type = LayerType::Linear;
      cur.W = m.cast<float>();
      pending = true;
    } else if (rest == "linear.bias") {
      const auto flat_b = m.cast<float>().eval();
      cur.b = Eigen::Map<const Eigen::VectorXf>(
          flat_b.data(), flat_b.size());
      pending = true;
    } else if (rest == "ln.gamma") {
      cur.type = LayerType::LayerNorm;
      const auto flat_gamma = m.cast<float>().eval();
      cur.gamma = Eigen::Map<const Eigen::VectorXf>(
          flat_gamma.data(), flat_gamma.size());
      pending = true;
    } else if (rest == "ln.beta") {
      const auto flat_beta = m.cast<float>().eval();
      cur.beta = Eigen::Map<const Eigen::VectorXf>(
          flat_beta.data(), flat_beta.size());
      pending = true;
    } else if (rest == "tanh") {
      cur.type = LayerType::Tanh;
      pending = true;
    } else if (rest == "relu") {
      cur.type = LayerType::ReLU;
      pending = true;
    } else {
      throw std::runtime_error("mlp: unknown record " + name);
    }
  }
  flush();

  if (mlp.layers.empty())
    throw std::runtime_error("mlp: checkpoint has no layer records");
  mlp.checkShapes();
  return mlp;
}

// A malformed checkpoint would otherwise surface as an out-of-bounds Eigen
// product in forward(), which Release builds do not check.
void MLP::checkShapes() const {
  Eigen::Index dim = -1;  // feature size flowing into the layer, once known
  for (size_t i = 0; i < layers.size(); ++i) {
    const Layer& l = layers[i];
    const std::string where = "mlp: layer " + std::to_string(i);
    switch (l.type) {
      case LayerType::Linear:
        if (l.W.size() == 0 || l.b.size() != l.W.rows())
          throw std::runtime_error(where +
                                   " needs a weight and a matching bias");
        if (dim >= 0 && l.W.cols() != dim)
          throw std::runtime_error(where + " input size does not match");
        dim = l.W.rows();
        break;
      case LayerType::LayerNorm:
        if (l.gamma.size() == 0 || l.beta.size() != l.gamma.size())
          throw std::runtime_error(where + " needs matching gamma and beta");
        if (dim >= 0 && l.gamma.size() != dim)
          throw std::runtime_error(where + " size does not match");
        dim = l.gamma.size();
        break;
      case LayerType::Tanh:
      case LayerType::ReLU:
        break;
    }
  }
}

MatF MLP::forward(const MatF& X) const {
  MatF h = X;
  for (const auto& layer : layers) {
    switch (layer.type) {
      case LayerType::Linear:
        h = (layer.W * h).colwise() + layer.b;
        break;
      case LayerType::LayerNorm: {
        // Per-sample (column) normalization, biased variance, torch semantics.
        const auto dim = static_cast<float>(h.rows());
        const Eigen::RowVectorXf mean = h.colwise().mean();
        h.rowwise() -= mean;
        const Eigen::RowVectorXf std_dev =
            (h.array().square().colwise().sum() / dim + kLayerNormEps).sqrt();
        h.array().rowwise() /= std_dev.array();
        h = ((h.array().colwise() * layer.gamma.array()).colwise() +
             layer.beta.array())
                .matrix();
        break;
      }
      case LayerType::Tanh:
        h = h.array().tanh().matrix();
        break;
      case LayerType::ReLU:
        h = h.cwiseMax(0.0f);
        break;
    }
  }
  return h;
}

int MLP::inputDim() const {
  for (const auto& l : layers)
    if (l.type == LayerType::Linear) return static_cast<int>(l.W.cols());
  return -1;
}

int MLP::outputDim() const {
  for (auto it = layers.rbegin(); it != layers.rend(); ++it)
    if (it->type == LayerType::Linear) return static_cast<int>(it->W.rows());
  return -1;
}

}  // namespace neupan
