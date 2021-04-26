#include "sampler.hh"
#include "mesh_adapter.hh"

#include <openvdb/Types.h>
#include <openvdb/tools/LevelSetFilter.h>
#include <openvdb/tools/LevelSetMorph.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Clip.h>

#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/partitioner.h>
#include <tbb/task_group.h>
#include <tbb/task_scheduler_init.h>

namespace flywave {
namespace voxelize {

class adjust_unclosest_face {
public:
  using int32_tree = openvdb::tree::Tree4<int32_t, 5, 4, 3>::Type;
  using int32_grid = openvdb::Grid<int32_tree>;

public:
  adjust_unclosest_face(
      std::vector<typename vertex_grid::TreeType::LeafNodeType *> &node,
      triangles_stream &stream, int32_grid &pri_grid, int32_grid &grid,
      std::vector<std::set<int32_t>> &polygons, vdb::math::Transform &transfrom)
      : _nodes(node), _polygons(polygons), _grid(grid), _pri_grid(pri_grid),
        _stream(stream), _transfrom(transfrom) {}

  void operator()(const tbb::blocked_range<size_t> &range) const {
    openvdb::tree::ValueAccessor<const int32_grid::TreeType> idxAcc(
        _grid.tree());
    openvdb::tree::ValueAccessor<const int32_grid::TreeType> pidxAcc(
        _pri_grid.tree());

    for (size_t n = range.begin(), N = range.end(); n < N; ++n) {
      for (auto it = _nodes[n]->beginValueOn(); it; ++it) {
        if (it.getValue() < 0)
          continue;

        std::set<int32_t> _tmp;
        std::set<int32_t> *_pri = &_tmp;
        _pri = &_polygons[size_t(idxAcc.getValue(it.getCoord()))];

        bool is_ng = true;
        bool is_inter = false;

        for (auto &pol : *_pri) {
          auto ftri = _stream.find_triangle_transfromed(pol);
          triangle3<double> tri(ftri[0], ftri[1], ftri[2]);

          auto pl = tri.plane();
          auto p = it.getCoord().asVec3d();

          p = p - pl.project_point(p);
          p.normalize();
          auto dot = pl.normal.dot(p);

          if (dot >= 0) {
            is_ng = false;
            break;
          }

          vdb::math::Ray<double> ray(it.getCoord().asVec3d(), -p);
          if (intersects_triangle(ray, tri[0], tri[1], tri[2])) {
            is_inter = true;
          }
        }

        if (is_ng && is_inter) {
          it.setValue(-it.getValue());
        }
      }
    }
  }

  bool intersects_triangle(const vdb::math::Ray<double> &ray,
                           const vdb::math::Vec3<double> &a,
                           const vdb::math::Vec3<double> &b,
                           const vdb::math::Vec3<double> &c) const {
    vdb::math::Vec3<double> target;
    bool backfaceCulling = false;
    auto edge1 = b - a;
    auto edge2 = c - a;
    auto normal = edge1.cross(edge2);

    auto DdN = ray.dir().dot(normal);
    int sign;

    if (DdN > 0) {
      if (backfaceCulling)
        return false;
      sign = 1;
    } else if (DdN < 0) {
      sign = -1;
      DdN = -DdN;
    } else {
      return false;
    }

    auto diff = ray.eye() - a;
    edge2 = diff.cross(edge2);
    auto DdQxE2 = sign * ray.dir().dot(edge2);

    if (DdQxE2 < 0) {
      return false;
    }

    auto DdE1xQ = sign * ray.dir().dot(edge1.cross(diff));

    if (DdE1xQ < 0) {
      return false;
    }

    if (DdQxE2 + DdE1xQ > DdN) {
      return false;
    }

    auto QdN = -sign * diff.dot(normal);

    if (QdN < 0) {
      return false;
    }

    target = ray.dir() * (QdN / DdN) + ray.eye();
    return true;
  }

private:
  std::vector<typename vertex_grid::TreeType::LeafNodeType *> &_nodes;
  std::vector<std::set<int32_t>> &_polygons;
  int32_grid &_grid;
  int32_grid &_pri_grid;
  triangles_stream &_stream;
  vdb::math::Transform &_transfrom;
};

void fix_unclosest_face(triangles_stream &stream, vertex_grid::Ptr ptr,
                        adjust_unclosest_face::int32_grid::Ptr _pri_index,
                        adjust_unclosest_face::int32_grid::Ptr _index,
                        std::vector<std::set<int32_t>> &polygons,
                        vdb::math::Transform &tran) {
  std::vector<typename vertex_grid::TreeType::LeafNodeType *> nodes;
  nodes.reserve(ptr->tree().leafCount());
  ptr->tree().getNodes(nodes);

  const tbb::blocked_range<size_t> nodeRange(0, nodes.size());

  adjust_unclosest_face op(nodes, stream, *_pri_index, *_index, polygons, tran);
  // op(nodeRange);
  tbb::parallel_for(nodeRange, op);

  openvdb::tools::signedFloodFill(ptr->tree());
}

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
    using DataTable =
        tbb::enumerable_thread_specific<typename VoxelizationDataType::Ptr>;
    openvdb::util::NullInterrupter interrupter;

    DataTable data;
    using Voxelizer = openvdb::tools::mesh_to_volume_internal::VoxelizePolygons<
        vertex_grid::TreeType, triangles_stream,
        openvdb::util::NullInterrupter>;

    const tbb::blocked_range<size_t> polygonRange(0, stream.polygonCount());

    tbb::parallel_for(polygonRange, Voxelizer(data, stream, &interrupter));

    for (typename DataTable::iterator i = data.begin(); i != data.end(); ++i) {
      VoxelizationDataType &dataItem = **i;
      openvdb::tools::mesh_to_volume_internal::combineData(
          distTree, indexTree, dataItem.distTree, dataItem.indexTree);
    }

    return std::tuple<vertex_grid::Ptr, int32_grid::Ptr>(_vertex_grid, _index);
  }
};

std::unique_ptr<vertext_sampler>
vertext_sampler::make_mesh_sampler(vdb::math::Transform::Ptr xform,
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
