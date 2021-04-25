
#include "voxel_pot.hh"

#include <openvdb/io/Compression.h>
#include <openvdb/tools/Clip.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/RayIntersector.h>

namespace flywave {
namespace voxelize {

voxel_pot::voxel_pot(vertex_grid::ptr vertex, pixel_grid::ptr pixel,
                     openvdb::math::Transform::Ptr res)
    : _resolution(res), _vertex(vertex), _pixel(pixel) {
  _vertex->set_grid_class(vdb::GRID_LEVEL_SET);
  _vertex->set_transform(_resolution);
}

bool voxel_pot::ray_test(const flywave::extray3<double> &ray, openvdb::Vec3d &p) {
  if (_vertex->empty())
    return false;
  vdb::tools::level_set_ray_intersector<vertex_grid> vray(*_vertex);

  return vray.intersects_WS(ray, p);
}

void voxel_pot::clear_unuse_materials() {
  std::map<material_id_t, bool> mapping;
  for (auto pt : _materials) {
    mapping.emplace(pt->_material_id, true);
  }

  auto iter = pixel_grid()->tree().begin_value_on();
  while (iter) {
    mapping[iter.get_value()._data._material_id] = false;
    ++iter;
  }

  for (auto pt : mapping) {
    if (pt.second == false)
      _materials.erase(_materials.begin() + pt.first);
  }
}

void voxel_pot_intersection(voxel_pot &tpot, voxel_pot &spot) {
  vdb::tools::csg_intersection(tpot.voxel_grid()->tree(),
                               spot.voxel_grid()->tree(), true);
}

void voxel_pot_union(voxel_pot &tpot, voxel_pot &spot) {
  auto pix_size = tpot.voxel_grid()->active_voxel_count();

  vdb::tools::csg_union(tpot.voxel_grid()->tree(), spot.voxel_grid()->tree(),
                        true);
  tpot.pixel_grid()->tree().merge(spot.pixel_grid()->tree());

  auto iter = spot.voxel_grid()->tree().begin_value_on();
  auto paccess = tpot.pixel_grid()->get_accessor();
  auto vaccess = tpot.voxel_grid()->get_accessor();

  while (iter) {
    if (!vaccess.is_value_on(iter.get_coord())) {
      paccess.set_value_off(iter.get_coord());
    }

    ++iter;
  }

  vdb::tools::prune_level_set(tpot.voxel_grid()->tree());
}

void voxel_pot_difference(voxel_pot &tpot, voxel_pot &spot) {
  vdb::tools::csg_difference(tpot.voxel_grid()->tree(),
                             spot.voxel_grid()->tree(), true);
}

void voxel_pot::composite(voxel_pot &pot, const composite_type &type) {
  // merge_materials(pot);
  _resolution = pot._resolution;

  if (!pot.is_empty()) {
    switch (type) {
    case composite_type::op_union:
      voxel_pot_union(*this, pot);
      break;

    case composite_type::op_intersection:
      voxel_pot_intersection(*this, pot);
      break;

    case composite_type::op_difference:
      voxel_pot_difference(*this, pot);
      break;
    }
  }
  _vertex->set_transform(_resolution);
}

} // namespace voxelize
} // namespace flywave
