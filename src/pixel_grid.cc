#include "pixel_grid.hh"

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

vdb_pixel_grid::vdb_pixel_grid() { openvdb::initialize(); }

vdb_pixel_grid::vdb_pixel_grid(vdb_pixel_grid *grid) {
  openvdb::initialize();
  _grid = grid->grid()->deepCopy();
}

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

} // namespace flywave
