#pragma once

#include "resolution.hh"
#include "triangle.hh"
#include "types.hh"

namespace flywave {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

class material_group;

class color_extract {
public:
  virtual color_type extract(const material_group &fgroup, const vdb::Vec2d &uv,
                             const triangle3<double> &tri) = 0;
};

} // namespace flywave
