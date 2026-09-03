#pragma once
// Private implementation header: pulls in the heavy JSON / Eigen / spdlog
// headers. Only src/utility/config.cpp (which performs the explicit template
// instantiations) may include this file; the public config.hpp stays light.

#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <ayanami/utility/config.hpp>

namespace ayanami {

namespace {

// IO traits: maps a C++ type to its JSON InType and back.
template <typename T> struct traits {
  using InType = T;
  using OutType = T;

  static std::optional<OutType> convert(const InType& in) { return in; }
  static InType invert(const OutType& value) { return value; }
};

// General Eigen matrix IO (Vector2d/3d/4d etc. as JSON number arrays).
template <typename T, int N, int M> struct traits<Eigen::Matrix<T, N, M>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Matrix<T, N, M>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != static_cast<size_t>(N * M)) {
      return std::nullopt;
    }
    return Eigen::Map<const OutType>(in.data());
  }

  static InType invert(const OutType& value) {
    return InType(value.data(), value.data() + N * M);
  }
};

// Eigen quaternion IO as [x, y, z, w].
template <typename T> struct traits<Eigen::Quaternion<T>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Quaternion<T>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != 4) {
      return std::nullopt;
    }
    return OutType(in.data()).normalized();
  }

  static InType invert(const OutType& value) {
    return InType{value.x(), value.y(), value.z(), value.w()};
  }
};

// Eigen isometry as TUM 7-tuple [x, y, z, qx, qy, qz, qw].
template <typename T>
struct traits<Eigen::Transform<T, 3, Eigen::Isometry>> {
  using InType = std::vector<double>;
  using OutType = Eigen::Transform<T, 3, Eigen::Isometry>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() != 7) {
      return std::nullopt;
    }
    OutType se3 = OutType::Identity();
    se3.translation() = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(in.data());
    se3.linear() =
        Eigen::Quaternion<T>(in.data() + 3).normalized().toRotationMatrix();
    return se3;
  }

  static InType invert(const OutType& value) {
    const Eigen::Matrix<T, 3, 1> t = value.translation();
    const Eigen::Quaternion<T> q(value.linear());
    return InType{t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
  }
};

// Trajectory of TUM 7-tuples.
template <> struct traits<std::vector<Eigen::Isometry3d>> {
  using InType = std::vector<double>;
  using OutType = std::vector<Eigen::Isometry3d>;

  static std::optional<OutType> convert(const InType& in) {
    if (in.size() % 7) {
      return std::nullopt;
    }
    OutType poses(in.size() / 7);
    for (size_t i = 0; i < poses.size(); i++) {
      poses[i].setIdentity();
      poses[i].translation() << in[i * 7], in[i * 7 + 1], in[i * 7 + 2];
      poses[i].linear() = Eigen::Quaterniond(in[i * 7 + 6], in[i * 7 + 3],
                                             in[i * 7 + 4], in[i * 7 + 5])
                              .normalized()
                              .toRotationMatrix();
    }
    return poses;
  }

  static InType invert(const OutType& value) {
    InType values(value.size() * 7);
    for (size_t i = 0; i < value.size(); i++) {
      const Eigen::Vector3d t = value[i].translation();
      const Eigen::Quaterniond q(value[i].linear());
      values[i * 7] = t.x();
      values[i * 7 + 1] = t.y();
      values[i * 7 + 2] = t.z();
      values[i * 7 + 3] = q.x();
      values[i * 7 + 4] = q.y();
      values[i * 7 + 5] = q.z();
      values[i * 7 + 6] = q.w();
    }
    return values;
  }
};

template <> struct traits<std::vector<Eigen::Vector2d>> {
  using InType = std::vector<std::vector<double>>;
  using OutType = std::vector<Eigen::Vector2d>;

  static std::optional<OutType> convert(const InType& input) {
    OutType output;
    output.reserve(input.size());
    for (const auto& point : input) {
      if (point.size() != 2) return std::nullopt;
      output.emplace_back(point[0], point[1]);
    }
    return output;
  }

  static InType invert(const OutType& value) {
    InType output;
    output.reserve(value.size());
    for (const auto& point : value) output.push_back({point.x(), point.y()});
    return output;
  }
};

} // namespace

template <typename T>
std::optional<T> Config::param(const std::string& module_name,
                               const std::string& param_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);

  const auto module = json.find(module_name);
  if (module == json.end()) {
    return std::nullopt;
  }

  const auto parameter = module->find(param_name);
  if (parameter == module->end()) {
    return std::nullopt;
  }

  return traits<T>::convert(parameter->get<typename traits<T>::InType>());
}

template <typename T>
T Config::param(const std::string& module_name, const std::string& param_name,
                const T& default_value) const {
  const auto found = param<T>(module_name, param_name);
  if (!found) {
    spdlog::warn("param {}/{} not found, using default value", module_name,
                 param_name);
    return default_value;
  }
  return *found;
}

template <typename T>
T Config::param_cast(const std::string& module_name,
                     const std::string& param_name) const {
  const auto found = param<T>(module_name, param_name);
  if (!found) {
    spdlog::critical("param {}/{} not found", module_name, param_name);
    abort();
  }
  return *found;
}

template <typename T>
void Config::overrideParam(const std::string& module_name,
                            const std::string& param_name, const T& value) {
  auto& json = std::any_cast<nlohmann::json&>(config);
  json[module_name][param_name] = traits<T>::invert(value);
}

#define DEFINE_CONFIG_IO_SPECIALIZATION(TYPE)                                  \
  template std::optional<TYPE> Config::param(                                  \
      const std::string& module_name, const std::string& param_name) const;    \
  template TYPE Config::param(const std::string&, const std::string&,          \
                              const TYPE&) const;                              \
  template TYPE Config::param_cast(const std::string&, const std::string&)     \
      const;                                                                   \
  template void Config::overrideParam(const std::string&, const std::string&, \
                                       const TYPE&);

} // namespace ayanami
