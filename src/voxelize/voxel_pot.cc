
#include "voxel_pot.hh"

#include <openvdb/io/Compression.h>
#include <openvdb/tools/Clip.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/RayIntersector.h>

namespace flywave {
namespace voxelize {

voxel_pot::voxel_pot(vertex_grid::Ptr vertex, pixel_grid::Ptr pixel,
                     vdb::math::Transform::Ptr res)
    : _resolution(res), _vertex(vertex), _pixel(pixel) {
  _vertex->setGridClass(openvdb::GRID_LEVEL_SET);
  _vertex->setTransform(_resolution);
}

bool voxel_pot::ray_test(const vdb::math::Ray<double> &ray, openvdb::Vec3d &p) {
  if (_vertex->empty())
    return false;

  openvdb::tools::LevelSetRayIntersector<vertex_grid> vray(*_vertex);

  return vray.intersectsWS(ray, p);
}

void voxel_pot::clear_unuse_materials() {
  std::map<material_id_t, bool> mapping;
  for (auto pt : _materials) {
    mapping.emplace(pt->_material_id, true);
  }

  auto iter = get_pixel_grid()->tree().beginValueOn();
  while (iter) {
    mapping[iter.getValue()._data._material_id] = false;
    ++iter;
  }

  for (auto pt : mapping) {
    if (pt.second == false)
      _materials.erase(_materials.begin() + pt.first);
  }
}

void voxel_pot_intersection(voxel_pot &tpot, voxel_pot &spot) {
  openvdb::tools::csgIntersection(tpot.get_voxel_grid()->tree(),
                                  spot.get_voxel_grid()->tree(), true);
}

void voxel_pot_union(voxel_pot &tpot, voxel_pot &spot) {
  openvdb::tools::csgUnion(tpot.get_voxel_grid()->tree(),
                           spot.get_voxel_grid()->tree(), true);
  tpot.get_pixel_grid()->tree().merge(spot.get_pixel_grid()->tree());

  auto iter = spot.get_voxel_grid()->tree().beginValueOn();
  auto paccess = tpot.get_pixel_grid()->getAccessor();
  auto vaccess = tpot.get_voxel_grid()->getAccessor();

  while (iter) {
    if (vaccess.isValueOn(iter.getCoord())) {
      paccess.setValue(iter.getCoord(), iter.getValue());
    } else {
      paccess.setValueOff(iter.getCoord());
    }
    ++iter;
  }

  openvdb::tools::pruneLevelSet(tpot.get_voxel_grid()->tree());
}

void voxel_pot_difference(voxel_pot &tpot, voxel_pot &spot) {
  openvdb::tools::csgDifference(tpot.get_voxel_grid()->tree(),
                                spot.get_voxel_grid()->tree(), true);
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
  _vertex->setTransform(_resolution);
}

} // namespace voxelize
} // namespace flywave
