#include "voxel_pixel_sampler.hh"
#include "projection.hh"
#include "sampler.hh"

#include <tbb/parallel_reduce.h>

namespace flywave {

void coloring(vertex_grid::Ptr vertex, pixel_grid::Ptr pixel,
              vertext_sampler::int32_grid::Ptr index,
              vdb::math::Transform::Ptr transform,
              const material_merge_transfrom &tmtl, mesh_adapter &_adapter,
              local_feature_id_t _local_feature_id);

std::shared_ptr<voxel_pixel>
voxel_pixel_sampler::apply(float precision, clip_box_createor &creator,
                           sampler_type type, material_merge_transfrom &tmtl,
                           openvdb::Mat4d matrix) {
  auto transform = _resolution.eval_resolution(precision);
  _adapter._stream->set_matrix(matrix);
  auto vsamper = vertext_sampler::make_mesh_sampler(transform, type);
  vertex_grid::Ptr vgrid;
  vertext_sampler::int32_grid::Ptr fgrid;
  std::tie(vgrid, fgrid) = vsamper->sampler(*(_adapter._stream), creator);
  std::vector<std::shared_ptr<material_data>> materials;
  for (auto &i : _adapter._materials) {
    materials.emplace_back(i.second.material_ptr());
  }

  tmtl.merge(std::move(materials));

  pixel_grid::Ptr pixel = pixel_grid::create();
  if (_adapter.has_materials()) {
   coloring(vgrid, pixel, fgrid, transform, tmtl, _adapter, _local_feature_id);
  }
    
  return std::make_shared<voxel_pixel>(vgrid, pixel, transform);
}

struct paint_color_on_surface {

  paint_color_on_surface(
      std::vector<typename vertext_sampler::int32_grid::TreeType::LeafNodeType
                      *> &nodes,
      vertex_grid::TreeType &vertex, pixel_grid::TreeType &pixel,
      vdb::math::Transform &transform, const material_merge_transfrom &tmtl,
      mesh_adapter &adapter, local_feature_id_t local_feature)
      : _nodes(nodes), _vertex(vertex), _pixel(&pixel), _transform(transform),
        _tmtl(tmtl), _adapter(adapter), _local_feature_id(local_feature) {}

  paint_color_on_surface(paint_color_on_surface &other, tbb::split)
      : _nodes(other._nodes), _vertex(other._vertex), _pixel(&_local_tree),
        _transform(other._transform), _tmtl(other._tmtl),
        _adapter(other._adapter), _local_feature_id(other._local_feature_id) {}

  void operator()(const tbb::blocked_range<size_t> &range) const {
    int32_t _triangle_index = -1;
    data_triangle _data_triangle;
    material_group const *_material_group;
    std::unique_ptr<uv_policy> uv_policy;

    openvdb::tree::ValueAccessor<const vertex_grid::TreeType> vaccess(_vertex);
    openvdb::tree::ValueAccessor<pixel_grid::TreeType> paccess(*_pixel);

    for (size_t n = range.begin(), N = range.end(); n < N; ++n) {
      auto &distNode = *_nodes[n];

      for (auto iter = distNode.beginValueOn(); iter; ++iter) {
        auto coord = iter.getCoord();
        if (!vaccess.isValueOn(coord))
          continue;

        auto value = iter.getValue();
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

          auto value = vaccess.getValue(coord);
          auto p = coord.asVec3d();
          auto tri_normal = tri.normal();
          p = p + tri_normal.unitSafe() * -value;

          auto color = sampler._extract->extract(
              *_material_group, uv_policy->eval_uv(_transform.indexToWorld(p)),
              _data_triangle._triangle);

          paccess.setValue(
              coord,
              pixel_data(_tmtl.new_material_id(_material_group->material_id()),
                         color, _local_feature_id));

        } else {
          paccess.setValue(
              coord,
              pixel_data(_tmtl.new_material_id(_material_group->material_id()),
                         _local_feature_id));
        }
      }
    }
  }

  void join(paint_color_on_surface &surface) { _pixel->merge(*surface._pixel); }

  std::vector<typename vertext_sampler::int32_grid::TreeType::LeafNodeType *>
      &_nodes;
  vertex_grid::TreeType &_vertex;
  pixel_grid::TreeType _local_tree;
  pixel_grid::TreeType *_pixel;
  vdb::math::Transform &_transform;
  const material_merge_transfrom &_tmtl;
  mesh_adapter &_adapter;
  local_feature_id_t _local_feature_id;
};

void coloring(vertex_grid::Ptr vertex, pixel_grid::Ptr pixel,
              vertext_sampler::int32_grid::Ptr index,
              vdb::math::Transform::Ptr transform,
              const material_merge_transfrom &tmtl, mesh_adapter &_adapter,
              local_feature_id_t _local_feature_id) {
  std::vector<typename vertext_sampler::int32_grid::TreeType::LeafNodeType *>
      nodes;
  nodes.reserve(index->tree().leafCount());
  index->tree().getNodes(nodes);

  const tbb::blocked_range<size_t> nodeRange(0, nodes.size());
  paint_color_on_surface op(nodes, vertex->tree(), pixel->tree(), *transform,
                            tmtl, _adapter, _local_feature_id);
  // op(nodeRange);
  tbb::parallel_reduce(nodeRange, op);
}
} // namespace flywave
