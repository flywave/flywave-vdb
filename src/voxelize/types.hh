#pragma once

#include <flywave/math/color.hh>
#include <flywave/math/plan.hh>
#include <flywave/math/ray.hh>
#include <flywave/math/vector_lib.hh>
#include <flywave/math/zero.hh>

namespace flywave {
namespace voxelize {

using index_type = vector3<uint32_t>;

using quad_type = vector4<int32_t>;

using triangle_type = vector3<int32_t>;

using vertext_type = vector3<float>;

using local_feature_id_t = uint16_t;
using globe_feature_id_t = uint64_t;

struct fmesh_tri_patch {
  vector3<float> p1;
  vector3<float> p2;
  vector3<float> p3;

  vector2<float> tp1;
  vector2<float> tp2;
  vector2<float> tp3;
};

using color_type = color4<uint8_t>;

using uv_type = vector2<float>;

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

template <typename scalar_type> class intersec_ray3 : public ray3<scalar_type> {
  using base = ray3<scalar_type>;

public:
  intersec_ray3(const vector3<scalar_type> &rorigin,
                const vector3<scalar_type> &rdirection)
      : base(rorigin, rdirection) {}

  bool distance_to_plane(const plan<scalar_type> &plane, scalar_type &value) {

    scalar_type denominator = plane.normal.dot(base::direction);

    if (denominator == 0) {

      if (plane.distance_to_point(base::origin) == 0) {

        value = 0.0;
        return true;
      }

      return false;
    }

    scalar_type t =
        -(base::origin.dot(plane.normal) + plane.distance) / denominator;

    if (t >= 0) {
      value = t;
      return true;
    } else {
      return false;
    }
  }
};

} // namespace voxelize
} // namespace flywave
