#include <flywave/voxelize/mesh_adapter.hh>
#include <flywave/voxelize/sampler.hh>

#include <flywave/vdb/tools/clip.hh>
#include <flywave/vdb/tools/mesh_to_volume.hh>
namespace flywave {
namespace voxelize {

class level_set_sampler : public vertext_sampler {
public:
  using vertext_sampler::vertext_sampler;

  future<vertex_grid::ptr, int32_grid::ptr>
  sampler(triangles_stream &stream, clip_box_createor &bc) override {
    return do_with(
        int32_grid::create(int32_t(vdb::util::INVALID_IDX)),
        [this, &stream, &bc](auto &grid) mutable {
          stream.set_transfrom(_xform);
          return vdb::tools::mesh_to_volume<vertex_grid>(stream, *_xform, 3.0f,
                                                         3.0f, 0, grid.get())
              .then([&grid, &bc, &stream, this](auto ptr) {
                assert(ptr);
#if false
              std::cout << "active voxel count:" << ptr->active_voxel_count()
                                                   << '\n';
#endif

                bbox3d clip_box;
                if (!bc(ptr, _xform, stream.compute_boundbox(), clip_box))
                  return make_ready_future<vertex_grid::ptr, int32_grid::ptr>(
                      ptr, grid);
                else
                  return make_ready_future<vertex_grid::ptr, int32_grid::ptr>(
                      vdb::tools::clip(*ptr, clip_box, true), grid);
              });
        });
  }
};

class surface_vertext_sampler : public vertext_sampler {
public:
  using vertext_sampler::vertext_sampler;

  future<vertex_grid::ptr, int32_grid::ptr>
  sampler(triangles_stream &stream, clip_box_createor &bc) override {
    return async([&stream, &bc, this] {
      stream.set_transfrom(_xform);
      vertex_grid::ptr _vertex_grid =
          vertex_grid::create(std::numeric_limits<float>::max());
      int32_grid::ptr _index =
          int32_grid::create(int32_t(vdb::util::INVALID_IDX));
      auto &indexTree = _index->tree();
      auto &distTree = _vertex_grid->tree();
      {
        vdb::util::null_interrupter interrupter;
        using voxelization_data_type =
            vdb::tools::mesh_to_volume_internal::voxelization_data<
                vertex_grid::tree_type>;
        using data_type = vdb::parallel::enumerable_thread_specific<
            typename voxelization_data_type::ptr>;

        data_type data;
        using Voxelizer =
            vdb::tools::mesh_to_volume_internal::voxelize_polygons<
                vertex_grid::tree_type, triangles_stream,
                vdb::util::null_interrupter>;

        const vdb::parallel::blocked_range<size_t> polygonRange(
            0, stream.polygon_count());

        vdb::parallel::parallel_for(
            polygonRange,
            Voxelizer(data, stream, _xform->copy(), &interrupter));

        for (typename data_type::iterator i = data.begin(); i != data.end();
             ++i) {
          voxelization_data_type &dataItem = ***i;
          vdb::tools::mesh_to_volume_internal::combine_data(
              distTree, indexTree, dataItem.distTree, dataItem.indexTree);
        }
      }
      return make_ready_future<vertex_grid::ptr, int32_grid::ptr>(_vertex_grid,
                                                                  _index);
    });
  }
};

std::unique_ptr<vertext_sampler>
vertext_sampler::make_mesh_sampler(vdb::math::transform::ptr xform,
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
