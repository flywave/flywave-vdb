#pragma once

#include "types.hh"

#include "mesh_adapter.hh"
#include "resolution.hh"
#include "uv_policy.hh"
#include "voxel_pixel.hh"

namespace flywave {

class voxel_pixel_sampler {
public:
  voxel_pixel_sampler(mesh_adapter &adapter,
                      local_feature_id_t local_feature = -1)
      : _adapter(adapter), _local_feature_id(local_feature) {}

  std::shared_ptr<voxel_pixel>
  apply(float precision, clip_box_createor &creator, sampler_type type,
        material_merge_transfrom &tmtl, openvdb::Mat4d matrix);

private:
  resolution _resolution;
  mesh_adapter &_adapter;
  local_feature_id_t _local_feature_id;
};

} // namespace flywave
