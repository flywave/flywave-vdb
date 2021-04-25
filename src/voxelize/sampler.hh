#pragma once

#include "mesh_adapter.hh"
#include "trees.hh"

#include <tuple>

namespace flywave {
namespace voxelize {

class vertext_sampler {
  using int32_tree = openvdb::tree::Tree4<int32_t, 5, 4, 3>::type;

public:
  using int32_grid = openvdb::Grid<int32_tree>;

public:
  vertext_sampler(openvdb::OPENVDB_VERSION_NAME::math::Transform::Ptr xform)
      : _xform(xform) {}

  virtual std::tuple<vertex_grid::Ptr, int32_grid::Ptr>
  sampler(triangles_stream &stream, clip_box_createor &bc) = 0;

  openvdb::OPENVDB_VERSION_NAME::math::Transform::Ptr transform() {
    return _xform;
  }

  static std::unique_ptr<vertext_sampler>
  make_mesh_sampler(openvdb::OPENVDB_VERSION_NAME::math::Transform::Ptr xform,
                    sampler_type type);

  virtual ~vertext_sampler() = default;

protected:
  openvdb::OPENVDB_VERSION_NAME::math::Transform::Ptr _xform;
};

} // namespace voxelize
} // namespace flywave
