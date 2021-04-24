#pragma once
#include <flywave/vdb/math/transform.hh>

namespace flywave {
namespace voxelize {

class resolution {
public:
  vdb::math::transform::ptr eval_resolution(const float &precision) const {
    return vdb::math::transform::create_linear_transform(1.0 / precision);
  }
};

} // namespace voxelize
} // namespace flywave
