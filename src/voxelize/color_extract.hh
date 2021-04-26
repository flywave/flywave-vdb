#pragma once

#include "resolution.hh"
#include "triangle.hh"
#include "types.hh"

namespace flywave {
namespace voxelize {

namespace vdb = openvdb::v8_1;

class material_group;

class color_extract {
public:
  virtual color_type extract(const material_group &fgroup, const vdb::Vec2d &uv,
                             const triangle3<float> &tri) = 0;
};

} // namespace voxelize
} // namespace flywave
