#pragma once

#include <openvdb/math/Transform.h>

namespace flywave {
namespace voxelize {

class resolution {
public:
  openvdb::OPENVDB_VERSION_NAME::math::Transform::Ptr
  eval_resolution(const float &precision) const {
    return openvdb::OPENVDB_VERSION_NAME::math::Transform::
        createLinearTransform(1.0 / precision);
  }
};

} // namespace voxelize
} // namespace flywave
