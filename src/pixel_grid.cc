#include "pixel_grid.hh"
#include "bbox.hh"

#include <openvdb/Types.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/LevelSetFilter.h>
#include <openvdb/tools/LevelSetMorph.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/VolumeToSpheres.h>

#include <cmath>

namespace flywave {

vdb_pixel_grid::vdb_pixel_grid() {}

vdb_pixel_grid::vdb_pixel_grid(vdb_pixel_grid *grid) {
  _grid = grid->grid()->deepCopy();
}

vdb_pixel_grid::vdb_pixel_grid(pixel_grid::Ptr grid) : _grid(grid) {}

vdb_pixel_grid::~vdb_pixel_grid() {}

pixel_grid::Ptr vdb_pixel_grid::grid() { return _grid; }

bool vdb_pixel_grid::read(const char *vFile) {
  openvdb::io::File file(vFile);

  file.open();

  openvdb::io::File::NameIterator nameIter = file.beginName();
  if (nameIter == file.endName()) {
    return false;
  }

  _grid = openvdb::gridPtrCast<pixel_grid>(file.readGrid(nameIter.gridName()));

  return true;
}

bool vdb_pixel_grid::write(const char *vFile) {
  openvdb::GridPtrVec grids;
  grids.push_back(_grid);

  openvdb::io::File file(vFile);
  file.write(grids);
  file.close();

  return true;
}

void vdb_pixel_grid::transform(vdb::math::Mat4d xform) {
  _grid->transform().postMult(xform);
}

void vdb_pixel_grid::set(const int i, const int j, const int k,
                         const pixel &v) {
  typename pixel_grid::Accessor accessor = _grid->getAccessor();
  openvdb::Coord ijk(i, j, k);
  accessor.setValue(ijk, v);
}

pixel vdb_pixel_grid::operator()(const int i, const int j, const int k) const {
  typename pixel_grid::Accessor accessor = _grid->getAccessor();
  openvdb::Coord ijk(i, j, k);
  return accessor.getValue(ijk);
}

pixel vdb_pixel_grid::operator()(const float i, const float j,
                                 const float k) const {
  typename pixel_grid::Accessor accessor = _grid->getAccessor();
  openvdb::Coord ijk(i, j, k);
  return accessor.getValue(ijk);
}

openvdb::Vec3d vdb_pixel_grid::bary_center() {
  if (grid() == nullptr)
    return openvdb::Vec3d(0, 0, 0);

  openvdb::CoordBBox bbox = grid()->evalActiveVoxelBoundingBox();
  return bbox.getCenter();
}

struct paint_texture_on_surface {

  paint_texture_on_surface(
      std::vector<typename pixel_grid::TreeType::LeafNodeType *> &stuff_nodes,
      const vdb::CoordBBox &tile_bbox, pixel_grid::TreeType &tile)
      : _tile(&tile), _stuff_nodes(stuff_nodes), _tile_bbox(tile_bbox) {}

  paint_texture_on_surface(paint_texture_on_surface &other, tbb::split)
      : _tile(other._tile), _stuff_nodes(other._stuff_nodes),
        _tile_bbox(other._tile_bbox) {}

  void operator()(const tbb::blocked_range<size_t> &range) const {
    auto orgx = _tile_bbox.max().x();
    auto to = _tile_bbox.min().x();

    bbox2<double> box(
        vdb::Vec2d{double(_tile_bbox.min().y()), double(_tile_bbox.min().z())},
        vdb::Vec2d{double(_tile_bbox.max().y()), double(_tile_bbox.max().z())});

    vdb::tree::ValueAccessor<pixel_grid::TreeType> paccess(*_tile);
    for (size_t n = range.begin(), N = range.end(); n < N; ++n) {
      auto &distNode = *_stuff_nodes[n];
      for (auto iter = distNode.beginValueOn(); iter; ++iter) {
        auto coord = iter.getCoord();
        if (!box.contains(vdb::Vec2d{double(coord.y()), double(coord.z())}))
          continue;

        auto ray = vdb::Coord(orgx, coord.y(), coord.z());
        bool hited = false;
        while (true) {
          if (ray.x() < to)
            break;
          if (!paccess.isValueOn(ray)) {
            if (hited)
              break;
            ray.x()--;
            continue;
          } else {
            hited = true;
            paccess.setValue(ray, iter.getValue());
          }
          ray.x()--;
        }
      }
    }
  }

  pixel_grid::TreeType *_tile;
  std::vector<typename pixel_grid::TreeType::LeafNodeType *> &_stuff_nodes;
  vdb::CoordBBox _tile_bbox;
};

void vdb_pixel_grid::paint_texture(pixel_grid::Ptr texvol) {
  std::vector<typename pixel_grid::TreeType::LeafNodeType *> nodes;
  nodes.reserve(texvol->tree().leafCount());
  texvol->tree().getNodes(nodes);

  vdb::CoordBBox tile_bbox = _grid->evalActiveVoxelBoundingBox();

  const tbb::blocked_range<size_t> nodeRange(0, nodes.size());
  paint_texture_on_surface op(nodes, tile_bbox, _grid->tree());

  // op(nodeRange);
  tbb::parallel_for(nodeRange, op);
}

} // namespace flywave
