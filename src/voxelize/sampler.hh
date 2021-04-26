#pragma once

#include "mesh_adapter.hh"
#include "trees.hh"

#include <tuple>

#include <openvdb/math/Transform.h>

namespace flywave {
namespace voxelize {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

class vertext_sampler {
  using int32_tree = vdb::tree::Tree4<int32_t, 5, 4, 3>::Type;

public:
  using int32_grid = vdb::Grid<int32_tree>;

public:
  vertext_sampler(vdb::math::Transform::Ptr xform) : _xform(xform) {}

  virtual std::tuple<vertex_grid::Ptr, int32_grid::Ptr>
  sampler(triangles_stream &stream, clip_box_createor &bc) = 0;

  vdb::math::Transform::Ptr transform() { return _xform; }

  static std::unique_ptr<vertext_sampler>
  make_mesh_sampler(vdb::math::Transform::Ptr xform, sampler_type type);

  virtual ~vertext_sampler() = default;

protected:
  vdb::math::Transform::Ptr _xform;
};

} // namespace voxelize
} // namespace flywave
