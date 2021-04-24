#pragma once 
#include <flywave/math/vector_lib.hh>
#include <flywave/math/triangle.hh>
#include <flywave/voxelize/resolution.hh>
#include <flywave/voxelize/types.hh>

#include <flywave/math/triangle.hh>

namespace flywave {
namespace voxelize {

class material_group; 
class color_extract {
public:
  virtual color_type extract(const material_group &fgroup,
                             const vector2<float> &uv,
                             const triangle3<float> &tri) = 0;
};

} // namespace voxelize
} // namespace flywave
