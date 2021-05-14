#pragma once

#include <openvdb/openvdb.h>
#include <openvdb/points/PointDataGrid.h>

#include "tolerance.hh"

namespace flywave {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

using vdb_grid = openvdb::GridBase;
using vdb_grid_ptr = openvdb::GridBase::Ptr;
using vdb_grid_const_ptr = openvdb::GridBase::ConstPtr;
using vdb_grid_ref = openvdb::GridBase &;
using vdb_grid_const_ref = const openvdb::GridBase &;

using vdb_grid_ptr_vec = openvdb::GridPtrVec;
using vdb_grid_ptr_vec_iterator = openvdb::GridPtrVec::iterator;
using vdb_grid_ptr_vec_const_iterator = openvdb::GridPtrVec::const_iterator;

using vdb_grid_const_ptr_vec = openvdb::GridCPtrVec;
using vdb_grid_const_ptr_vec_iterator = openvdb::GridCPtrVec::iterator;
using vdb_grid_const_ptr_vec_const_iterator = openvdb::GridCPtrVec::const_iterator;

using scalar_grid_types =
    vdb::TypeList<openvdb::BoolGrid, openvdb::FloatGrid, openvdb::DoubleGrid,
                  openvdb::Int32Grid, openvdb::Int64Grid>;

using numeric_grid_types =
    vdb::TypeList<openvdb::FloatGrid, openvdb::DoubleGrid, openvdb::Int32Grid,
                  openvdb::Int64Grid>;

using real_grid_types = vdb::TypeList<openvdb::FloatGrid, openvdb::DoubleGrid>;

using vec3_grid_types =
    vdb::TypeList<openvdb::Vec3SGrid, openvdb::Vec3DGrid, openvdb::Vec3IGrid>;

using point_grid_types = vdb::TypeList<vdb::points::PointDataGrid>;

using volume_grid_types = scalar_grid_types::Append<vec3_grid_types>;

using all_grid_types = volume_grid_types::Append<point_grid_types>;

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

  openvdb::Vec2d tp1;
  openvdb::Vec2d tp2;
  openvdb::Vec2d tp3;
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

} // namespace flywave
