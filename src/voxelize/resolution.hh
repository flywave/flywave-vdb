#pragma once

#include <openvdb/math/Transform.h>

namespace flywave {
namespace voxelize {

namespace vdb = openvdb::v8_1;

class resolution {
public:
  vdb::math::Transform::Ptr eval_resolution(const float &precision) const {
    return vdb::math::Transform::createLinearTransform(1.0 / precision);
  }
};

} // namespace voxelize
} // namespace flywave
