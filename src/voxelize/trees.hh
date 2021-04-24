#pragma once
#include <flywave/voxelize/pixel.hh>

#include <flywave/vdb/grid.hh>
#include <flywave/vdb/tree/tree.hh>
#include <flywave/voxelize/types.hh>

namespace flywave {
namespace voxelize {

using float_tree = vdb::tree::tree4<float, 5, 4, 3>::type;
using pixel_tree = vdb::tree::tree4<pixel, 5, 4, 3>::type;

using pixel_grid = vdb::grid<pixel_tree>;
using float_grid = vdb::grid<float_tree>;

using vertex_grid = float_grid;

class clip_box_createor {
public:
  virtual bool operator()(vertex_grid::ptr vertex,
                          vdb::math::transform::ptr resolution,
                          const bbox3d &sbox, bbox3d &cbox) = 0;
};

} // namespace voxelize
} // namespace flywave
