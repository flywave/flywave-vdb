#pragma once

#include <openvdb/Grid.h>
#include <openvdb/Types.h>
#include <openvdb/math/Transform.h>
#include <openvdb/tree/Tree.h>

#include "pixel.hh"
#include "types.hh"

namespace flywave {
namespace voxelize {

using float_tree = openvdb::tree::Tree4<float, 5, 4, 3>::Type;
using pixel_tree = openvdb::tree::Tree4<pixel, 5, 4, 3>::Type;

using pixel_grid = openvdb::Grid<pixel_tree>;
using float_grid = openvdb::Grid<float_tree>;

using vertex_grid = float_grid;

class clip_box_createor {
public:
  virtual bool
  operator()(vertex_grid::Ptr vertex,
             openvdb::OPENVDB_VERSION_NAME::math::Transform::Ptr resolution,
             const openvdb::BBoxd &sbox, openvdb::BBoxd &cbox) = 0;
};

} // namespace voxelize
} // namespace flywave
