#pragma once

#include <openvdb/Types.h>

#include "tolerance.hh"

namespace flywave {
namespace voxelize {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

using index_type = vdb::math::Vec3<uint32_t>;

using quad_type = vdb::math::Vec4<uint32_t>;

using triangle_type = vdb::math::Vec3<uint32_t>;

using vertext_type = vdb::math::Vec3<float>;

using local_feature_id_t = uint16_t;
using globe_feature_id_t = uint64_t;

struct fmesh_tri_patch {
  openvdb::Vec3d p1;
  openvdb::Vec3d p2;
  openvdb::Vec3d p3;

  openvdb::Vec3d tp1;
  openvdb::Vec3d tp2;
  openvdb::Vec3d tp3;
};

using color_type = vdb::math::Vec4<uint8_t>;

using uv_type = vdb::math::Vec2<float>;

enum class sampler_type { level_set, surface };

using material_id_t = uint8_t;
using face_index_t = uint32_t;

template <typename T> struct approx_value {
  inline T operator()(T value) const {
    T c = std::ceil(value);
    if (vdb::math::isApproxEqual(float(tol), float(c - value)))
      return c;
    return value;
  }

  T tol = tolerance<T>();
};

} // namespace voxelize
} // namespace flywave
