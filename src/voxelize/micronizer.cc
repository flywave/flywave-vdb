#include <flywave/voxelize/micronizer.hh>
#include <flywave/voxelize/projection.hh>
#include <flywave/voxelize/sampler.hh>

namespace flywave {
namespace voxelize {

void coloring(vertex_grid::ptr vertex, pixel_grid::ptr pixel,
              vertext_sampler::int32_grid::ptr index,
              vdb::math::transform::ptr transform,
              const material_merge_transfrom &tmtl, mesh_adapter &_adapter,
              local_feature_id_t _local_feature_id);

voxel_pot micronizer::micronize(float precision, clip_box_createor &creator,
                                sampler_type type,
                                material_merge_transfrom &tmtl,
                                matrix44<double> matrix) {
  auto transform = _resolution.eval_resolution(precision);
  _adapter._stream->set_matrix(matrix);
  return do_with(vertext_sampler::make_mesh_sampler(transform, type),
                 [this, &tmtl, &creator, transform](auto &vsamper) {
                   return vsamper->sampler(*(_adapter._stream), creator)
                       .then([this, &tmtl, transform](auto vgrid, auto fgrid) {
                         std::vector<shared_ptr<material_data>> materials;
                         for (auto &i : _adapter._materials) {
                           materials.emplace_back(i.second.material_ptr());
                         }

                         tmtl.merge(std::move(materials));

                         pixel_grid::ptr pixel = pixel_grid::create();
                         coloring(vgrid, pixel, fgrid, transform, tmtl,
                                  _adapter, _local_feature_id);

                         return voxel_pot(vgrid, pixel, transform);
                       });
                 });
}

struct paint_color_on_surface {

  paint_color_on_surface(
      std::vector<
          typename vertext_sampler::int32_grid::TreeType::leaf_node_type *>
          &nodes,
      vertex_grid::TreeType &vertex, pixel_grid::TreeType &pixel,
      vdb::math::transform &transform, const material_merge_transfrom &tmtl,
      mesh_adapter &adapter, local_feature_id_t local_feature)
      : _nodes(nodes), _vertex(vertex), _pixel(&pixel), _transform(transform),
        _tmtl(tmtl), _adapter(adapter), _local_feature_id(local_feature) {}

  paint_color_on_surface(paint_color_on_surface &other, vdb::parallel::split)
      : _nodes(other._nodes), _vertex(other._vertex), _pixel(&_local_tree),
        _transform(other._transform), _tmtl(other._tmtl),
        _adapter(other._adapter), _local_feature_id(other._local_feature_id) {}

  void operator()(const vdb::parallel::blocked_range<size_t> &range) const {
    int32_t _triangle_index = -1;
    data_triangle _data_triangle;
    material_group const *_material_group;
    std::unique_ptr<st_policy> uv_policy;

    vdb::tree::value_accessor<const vertex_grid::TreeType> vaccess(_vertex);
    vdb::tree::value_accessor<pixel_grid::TreeType> paccess(*_pixel);

    for (size_t n = range.begin(), N = range.end(); n < N; ++n) {
      auto &distNode = *_nodes[n];

      for (auto iter = distNode.begin_value_on(); iter; ++iter) {
        auto coord = iter.get_coord();
        if (!vaccess.is_value_on(coord))
          continue;

        auto value = iter.get_value();
        auto keep_old = true;
        if (_triangle_index != value) {
          _triangle_index = value;
          _data_triangle = _adapter._stream->find_triangle(value);
          _material_group =
              &_adapter.find_material(_data_triangle._material_id);
          if (_material_group->has_texture_sampler()) {
            uv_policy = _material_group->sampler()._policy->make_shared();
          }
          keep_old = false;
        }

        if (_material_group->has_texture_sampler()) {
          auto &sampler = _material_group->sampler();
          triangle3<double> tri(_data_triangle._triangle[0],
                                _data_triangle._triangle[1],
                                _data_triangle._triangle[2]);
          if (!keep_old) {
            uv_policy->start_triangle(_triangle_index, tri);
          }

          auto value = vaccess.get_value(coord);
          auto p = coord.to_float3();
          auto tri_normal = tri.normal();
          p = p + tri_normal.normalized() * -value;

          auto color = sampler._extract->extract(
              *_material_group,
              uv_policy->eval_uv(_transform.index_to_world(p)),
              _data_triangle._triangle);

          paccess.set_value(
              coord,
              pixel_data(_tmtl.new_material_id(_material_group->material_id()),
                         color, _local_feature_id));

        } else {
          paccess.set_value(
              coord,
              pixel_data(_tmtl.new_material_id(_material_group->material_id()),
                         _local_feature_id));
        }
      }
    }
  }

  void join(paint_color_on_surface &surface) { _pixel->merge(*surface._pixel); }

  std::vector<typename vertext_sampler::int32_grid::TreeType::leaf_node_type *>
      &_nodes;
  vertex_grid::TreeType &_vertex;
  pixel_grid::TreeType _local_tree;
  pixel_grid::TreeType *_pixel;
  vdb::math::transform &_transform;
  const material_merge_transfrom &_tmtl;
  mesh_adapter &_adapter;
  local_feature_id_t _local_feature_id;
};

void coloring(vertex_grid::ptr vertex, pixel_grid::ptr pixel,
              vertext_sampler::int32_grid::ptr index,
              vdb::math::transform::ptr transform,
              const material_merge_transfrom &tmtl, mesh_adapter &_adapter,
              local_feature_id_t _local_feature_id) {

  std::vector<typename vertext_sampler::int32_grid::TreeType::leaf_node_type *>
      nodes;
  nodes.reserve(index->tree().leaf_count());
  index->tree().get_nodes(nodes);

  const vdb::parallel::blocked_range<size_t> nodeRange(0, nodes.size());
  paint_color_on_surface op(nodes, vertex->tree(), pixel->tree(), *transform,
                            tmtl, _adapter, _local_feature_id);
  // op(nodeRange);
  vdb::parallel::parallel_reduce(nodeRange, op);
}

} // namespace voxelize
} // namespace flywave
