#pragma once

#include <flywave/voxelize/mesh_adapter.hh>

#include <flywave/voxelize/trees.hh>

namespace flywave {
namespace voxelize {

class vertext_sampler {
  using int32_tree = vdb::tree::tree4<int32_t, 5, 4, 3>::type;

public:
  using int32_grid = vdb::grid<int32_tree>;

public:
  vertext_sampler(vdb::math::transform::ptr xform) : _xform(xform) {}

  virtual future<vertex_grid::ptr, int32_grid::ptr>
  sampler(triangles_stream &stream, clip_box_createor &bc) = 0;

  vdb::math::transform::ptr transform() { return _xform; }

  static std::unique_ptr<vertext_sampler>
  make_mesh_sampler(vdb::math::transform::ptr xform, sampler_type type);

  virtual ~vertext_sampler() = default;

protected:
  vdb::math::transform::ptr _xform;
};

} // namespace voxelize
} // namespace flywave
