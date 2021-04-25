#pragma once
#include <flywave/voxelize/types.hh>

#include <flywave/voxelize/mesh_adapter.hh>
#include <flywave/voxelize/resolution.hh>
#include <flywave/voxelize/st_policy.hh>

#include <flywave/voxelize/voxel_pot.hh>

namespace flywave {
namespace voxelize {

class micronizer {
public:
  micronizer(mesh_adapter &adapter, local_feature_id_t local_feature = -1)
      : _adapter(adapter), _local_feature_id(local_feature) {}

  voxel_pot micronize(float precision, clip_box_createor &creator,
                      sampler_type type, material_merge_transfrom &tmtl,
                      openvdb::Mat4d matrix);

private:
  resolution _resolution;
  mesh_adapter &_adapter;
  local_feature_id_t _local_feature_id;
};

} // namespace voxelize
} // namespace flywave
