#include "sampler.hh"
#include "mesh_adapter.hh"

#include <openvdb/tools/Clip.h>
#include <openvdb/tools/MeshToVolume.h>

#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/partitioner.h>
#include <tbb/task_group.h>
#include <tbb/task_scheduler_init.h>

namespace flywave {
namespace voxelize {

class level_set_sampler : public vertext_sampler {
public:
  using vertext_sampler::vertext_sampler;

  std::tuple<vertex_grid::Ptr, int32_grid::Ptr>
  sampler(triangles_stream &stream, clip_box_createor &bc) override {

    auto grid = int32_grid::create(int32_t(openvdb::util::INVALID_IDX));
    stream.set_transfrom(_xform);
    auto ptr = openvdb::tools::meshToVolume<vertex_grid>(stream, *_xform, 3.0f,
                                                         3.0f, 0, grid.get());
    assert(ptr);
    openvdb::BBoxd clip_box;
    if (!bc(ptr, _xform, stream.compute_boundbox(), clip_box))
      return std::tuple<vertex_grid::Ptr, int32_grid::Ptr>(ptr, grid);
    else
      return std::tuple<vertex_grid::Ptr, int32_grid::Ptr>(
          openvdb::tools::clip(*ptr, clip_box, true), grid);
  }
};

class surface_vertext_sampler : public vertext_sampler {
public:
  using vertext_sampler::vertext_sampler;

  std::tuple<vertex_grid::Ptr, int32_grid::Ptr>
  sampler(triangles_stream &stream, clip_box_createor &bc) override {
    stream.set_transfrom(_xform);
    vertex_grid::Ptr _vertex_grid =
        vertex_grid::create(std::numeric_limits<float>::max());
    int32_grid::Ptr _index =
        int32_grid::create(int32_t(openvdb::util::INVALID_IDX));

    auto &indexTree = _index->tree();
    auto &distTree = _vertex_grid->tree();

    using VoxelizationDataType =
        openvdb::tools::mesh_to_volume_internal::VoxelizationData<
            vertex_grid::TreeType>;
   using DataTable = tbb::enumerable_thread_specific<typename VoxelizationDataType::Ptr>;
    openvdb::util::NullInterrupter interrupter;

    DataTable data;
    using Voxelizer = openvdb::tools::mesh_to_volume_internal::VoxelizePolygons<
        vertex_grid::TreeType, triangles_stream,
        openvdb::util::NullInterrupter>;

    const tbb::blocked_range<size_t> polygonRange(0, stream.polygon_count());

    tbb::parallel_for(polygonRange,
                      Voxelizer(data, _xform->copy(), &interrupter));

    for (typename DataTable::iterator i = data.begin(); i != data.end(); ++i) {
      VoxelizationDataType &dataItem = **i;
      openvdb::tools::mesh_to_volume_internal::combineData(
          distTree, indexTree, dataItem.distTree, dataItem.indexTree);
    }

    return std::tuple<vertex_grid::Ptr, int32_grid::Ptr>(_vertex_grid, _index);
  }
};

std::unique_ptr<vertext_sampler> vertext_sampler::make_mesh_sampler(
    openvdb::OPENVDB_VERSION_NAME::math::Transform::Ptr xform,
    sampler_type type) {
  switch (type) {
  case sampler_type::level_set:
    return std::make_unique<level_set_sampler>(xform);

  case sampler_type::surface:
    return std::make_unique<surface_vertext_sampler>(xform);
  }
  return nullptr;
}

} // namespace voxelize
} // namespace flywave
