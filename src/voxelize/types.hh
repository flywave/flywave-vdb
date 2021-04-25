#pragma once

#include <Eigen/Geometry>

namespace flywave {
namespace voxelize {

using index_type = Eigen::Matrix<uint32_t, 3, 1>;

using quad_type = Eigen::Matrix<int32_t, 3, 1>;

using triangle_type = Eigen::Matrix<int32_t, 3, 1>;

using vertext_type = Eigen::Matrix<float, 3, 1>;

using local_feature_id_t = uint16_t;
using globe_feature_id_t = uint64_t;

struct fmesh_tri_patch {
  Eigen::Matrix<float, 3, 1> p1;
  Eigen::Matrix<float, 3, 1> p2;
  Eigen::Matrix<float, 3, 1> p3;

  Eigen::Matrix<float, 3, 1> tp1;
  Eigen::Matrix<float, 3, 1> tp2;
  Eigen::Matrix<float, 3, 1> tp3;
};

using color_type = Eigen::Matrix<uint8_t, 4, 1>;

using uv_type = Eigen::Matrix<float, 2, 1>;

enum class sampler_type { level_set, surface };

using material_id_t = uint8_t;
using face_index_t = uint32_t;

template <typename T> struct approx_value {
  inline T operator()(T value) const {
    T c = std::ceil(value);
    if (::flywave::math::is_approx_equal(float(tol), float(c - value)))
      return c;
    return value;
  }

  T tol = tolerance<T>();
};

} // namespace voxelize
} // namespace flywave
