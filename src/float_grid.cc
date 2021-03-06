#include "float_grid.hh"

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

vdb_float_grid::vdb_float_grid() {}

vdb_float_grid::vdb_float_grid(vdb_float_grid *grid) {
  _grid = grid->grid()->deepCopy();
  _display = grid->display().duplicate();
}

vdb_float_grid::vdb_float_grid(float_grid::Ptr grid) : _grid(grid) {}

vdb_float_grid::~vdb_float_grid() {}

openvdb::FloatGrid::Ptr vdb_float_grid::grid() { return _grid; }

bool vdb_float_grid::has_grid() const { return _grid != nullptr; }

bool vdb_float_grid::read(const char *vFile) {
  openvdb::io::File file(vFile);

  file.open();

  openvdb::io::File::NameIterator nameIter = file.beginName();
  if (nameIter == file.endName()) {
    return false;
  }

  _grid = openvdb::gridPtrCast<openvdb::FloatGrid>(
      file.readGrid(nameIter.gridName()));

  return true;
}

bool vdb_float_grid::write(const char *vFile) {
  openvdb::GridPtrVec grids;
  grids.push_back(_grid);

  openvdb::io::File file(vFile);
  file.write(grids);
  file.close();

  return true;
}

bool vdb_float_grid::create_from_mesh(mesh_data vMesh, double voxelSize,
                                      double bandwidth) {
  if (!vMesh.is_valid()) {
    return false;
  }

  vdb::math::Transform xform;
  xform.preScale(voxelSize);

  auto vertices = vMesh.vertices();
  auto faces = vMesh.mtl_faces_map()[none_mtl_id];

  openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec3I>
      mesh(vertices, faces);
  _grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
      mesh, xform, static_cast<float>(bandwidth), static_cast<float>(bandwidth),
      0, NULL);

  _display = vMesh;

  return true;
}

bool vdb_float_grid::create_from_points(vdb_particle vPoints, double voxelSize,
                                        double bandwidth) {
  if (!vPoints.is_valid()) {
    return false;
  }

  _grid = openvdb::createLevelSet<openvdb::FloatGrid>(voxelSize, bandwidth);
  openvdb::tools::ParticlesToLevelSet<openvdb::FloatGrid> raster(*_grid);

  vdb::math::Transform::Ptr xform =
      vdb::math::Transform::createLinearTransform(voxelSize);
  _grid->setTransform(xform);

  raster.setGrainSize(1);
  raster.rasterizeSpheres(vPoints);
  raster.finalize();

  return true;
}

void vdb_float_grid::rebuild(float iso, float exWidth, float inWidth) {
  _grid = openvdb::tools::levelSetRebuild(*_grid, 0., exWidth, inWidth);
}

void vdb_float_grid::transform(vdb::math::Mat4d xform) {
  _grid->transform().postMult(xform);
}

void vdb_float_grid::boolean_union(vdb_float_grid vAdd) {
  auto csgGrid = vAdd.grid();

  const vdb::math::Transform &sourceXform = csgGrid->transform(),
                             &targetXform = _grid->transform();

  openvdb::FloatGrid::Ptr cGrid =
      openvdb::createLevelSet<openvdb::FloatGrid>(_grid->voxelSize()[0]);
  cGrid->transform() = _grid->transform();

  openvdb::Mat4R xform =
      sourceXform.baseMap()->getAffineMap()->getMat4() *
      targetXform.baseMap()->getAffineMap()->getMat4().inverse();

  openvdb::tools::GridTransformer transformer_(xform);

  transformer_.transformGrid<openvdb::tools::BoxSampler, openvdb::FloatGrid>(
      *csgGrid, *cGrid);

  openvdb::tools::csgUnion(*_grid, *cGrid, true);
}

void vdb_float_grid::boolean_intersection(vdb_float_grid vIntersect) {
  auto csgGrid = vIntersect.grid();

  const vdb::math::Transform &sourceXform = csgGrid->transform(),
                             &targetXform = _grid->transform();

  openvdb::FloatGrid::Ptr cGrid =
      openvdb::createLevelSet<openvdb::FloatGrid>(_grid->voxelSize()[0]);
  cGrid->transform() = _grid->transform();

  openvdb::Mat4R xform =
      sourceXform.baseMap()->getAffineMap()->getMat4() *
      targetXform.baseMap()->getAffineMap()->getMat4().inverse();

  openvdb::tools::GridTransformer transformer_(xform);

  transformer_.transformGrid<openvdb::tools::BoxSampler, openvdb::FloatGrid>(
      *csgGrid, *cGrid);

  openvdb::tools::csgIntersection(*_grid, *cGrid, true);
}

void vdb_float_grid::boolean_difference(vdb_float_grid vSubtract) {
  auto csgGrid = vSubtract.grid();

  const vdb::math::Transform &sourceXform = csgGrid->transform(),
                             &targetXform = _grid->transform();

  openvdb::FloatGrid::Ptr cGrid =
      openvdb::createLevelSet<openvdb::FloatGrid>(_grid->voxelSize()[0]);
  cGrid->transform() = _grid->transform();

  openvdb::Mat4R xform =
      sourceXform.baseMap()->getAffineMap()->getMat4() *
      targetXform.baseMap()->getAffineMap()->getMat4().inverse();

  openvdb::tools::GridTransformer transformer_(xform);

  transformer_.transformGrid<openvdb::tools::BoxSampler, openvdb::FloatGrid>(
      *csgGrid, *cGrid);

  openvdb::tools::csgDifference(*_grid, *cGrid, true);
}

void vdb_float_grid::offset(double amount) {
  openvdb::tools::LevelSetFilter<openvdb::FloatGrid> filter(*_grid);

  filter.setGrainSize(1);

  amount = amount * -1;

  filter.offset((float)amount);
}

void vdb_float_grid::offset(double amount, vdb_float_grid vMask, double min,
                            double max, bool invert) {
  openvdb::tools::LevelSetFilter<openvdb::FloatGrid> filter(*_grid);

  filter.invertMask(invert);
  filter.setMaskRange((float)min, (float)max);
  filter.setGrainSize(1);

  openvdb::Grid<openvdb::FloatTree> mMask(*vMask.grid());

  amount = amount * -1;

  filter.offset((float)amount, &mMask);
}

void vdb_float_grid::smooth(int type, int iterations, int width) {
  openvdb::tools::LevelSetFilter<openvdb::FloatGrid> filter(*_grid);
  filter.setGrainSize(1);

  for (int i = 0; i < iterations; i++) {
    switch (type) {
    case 0:
      filter.gaussian(width);
      break;
    case 1:
      filter.laplacian();
      break;
    case 2:
      filter.mean(width);
      break;
    case 3:
      filter.median(width);
      break;
    case 4:
      filter.meanCurvature();
      break;
    default:
      filter.laplacian();
      break;
    }
  }
}

void vdb_float_grid::smooth(int type, int iterations, int width,
                            vdb_float_grid vMask, double min, double max,
                            bool invert) {
  openvdb::tools::LevelSetFilter<openvdb::FloatGrid> filter(*_grid);

  filter.invertMask(invert);
  filter.setMaskRange((float)min, (float)max);
  filter.setGrainSize(1);

  openvdb::Grid<openvdb::FloatTree> mMask(*vMask.grid());

  for (int i = 0; i < iterations; i++) {
    switch (type) {
    case 0:
      filter.gaussian(width, &mMask);
      break;
    case 1:
      filter.laplacian(&mMask);
      break;
    case 2:
      filter.mean(width, &mMask);
      break;
    case 3:
      filter.median(width, &mMask);
      break;
    case 4:
      filter.meanCurvature();
      break;
    default:
      filter.laplacian(&mMask);
      break;
    }
  }
}

void vdb_float_grid::blend(vdb_float_grid bGrid, double bPosition,
                           double bEnd) {
  openvdb::tools::LevelSetMorphing<openvdb::FloatGrid> morph(*_grid,
                                                             *bGrid.grid());
  morph.setSpatialScheme(vdb::math::HJWENO5_BIAS);
  morph.setTemporalScheme(vdb::math::TVD_RK3);
  morph.setTrackerSpatialScheme(vdb::math::HJWENO5_BIAS);
  morph.setTrackerTemporalScheme(vdb::math::TVD_RK2);
  morph.setGrainSize(1);

  double bStart = bPosition * bEnd;
  morph.advect(bStart, bEnd);
}

void vdb_float_grid::blend(vdb_float_grid bGrid, double bPosition, double bEnd,
                           vdb_float_grid vMask, double mMin, double mMax,
                           bool invert) {
  openvdb::tools::LevelSetMorphing<openvdb::FloatGrid> morph(*_grid,
                                                             *bGrid.grid());
  morph.setSpatialScheme(vdb::math::HJWENO5_BIAS);
  morph.setTemporalScheme(vdb::math::TVD_RK3);
  morph.setTrackerSpatialScheme(vdb::math::HJWENO5_BIAS);
  morph.setTrackerTemporalScheme(vdb::math::TVD_RK2);

  morph.setAlphaMask(*vMask.grid());
  morph.invertMask(invert);
  morph.setMaskRange((float)mMin, (float)mMax);
  morph.setGrainSize(1);

  double bStart = bPosition * bEnd;
  morph.advect(bStart, bEnd);
}

void vdb_float_grid::closest_point(std::vector<openvdb::Vec3R> &points,
                                   std::vector<float> &distances) {
  auto csp =
      openvdb::tools::ClosestSurfacePoint<openvdb::FloatGrid>::create(*_grid);
  csp->searchAndReplace(points, distances);
}

openvdb::Vec3d vdb_float_grid::bary_center() {
  if (grid() == nullptr)
    return openvdb::Vec3d(0, 0, 0);

  openvdb::CoordBBox bbox = grid()->evalActiveVoxelBoundingBox();
  return bbox.getCenter();
}

mesh_data vdb_float_grid::display() { return _display; }

void vdb_float_grid::update_display() {
  using openvdb::Index64;

  openvdb::tools::VolumeToMesh mesher(
      _grid->getGridClass() == openvdb::GRID_LEVEL_SET ? 0.0 : 0.01);
  mesher(*_grid);

  _display.clear();

  for (Index64 n = 0, N = mesher.pointListSize(); n < N; ++n) {
    auto v = mesher.pointList()[n];
    _display.add_vertice(v);
  }

  openvdb::tools::PolygonPoolList &polygonPoolList = mesher.polygonPoolList();

  for (Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
    const openvdb::tools::PolygonPool &polygons = polygonPoolList[n];
    for (Index64 i = 0, I = polygons.numTriangles(); i < I; ++i) {
      auto face = polygons.triangle(i);
      _display.add_mtl_face(none_mtl_id, face);
    }
  }
}

void vdb_float_grid::update_display(double isovalue, double adaptivity) {
  isovalue /= _grid->voxelSize().x();

  std::vector<openvdb::Vec3s> points;
  std::vector<openvdb::Vec4I> quads;
  std::vector<openvdb::Vec3I> triangles;

  openvdb::tools::volumeToMesh<openvdb::FloatGrid>(*_grid, points, triangles,
                                                   quads, isovalue, adaptivity);

  _display.clear();

  _display.add_vertice(points);
  _display.add_mtl_faces(none_mtl_id, triangles);
}

float *vdb_float_grid::get_mesh_vertices() {
  auto vertices = _display.vertices();

  _vertex_count = vertices.size() * 3;

  float *verticeArray =
      reinterpret_cast<float *>(malloc(_vertex_count * sizeof(float)));

  int i = 0;
  for (auto it = vertices.begin(); it != vertices.end(); ++it) {
    verticeArray[i] = it->x();
    verticeArray[i + 1] = it->y();
    verticeArray[i + 2] = it->z();
    i += 3;
  }

  return verticeArray;
}

int *vdb_float_grid::get_mesh_faces() {
  auto faces = _display.mtl_faces_map()[none_mtl_id];

  _face_count = faces.size() * 3;

  int *faceArray = reinterpret_cast<int *>(malloc(_face_count * sizeof(int)));

  int i = 0;
  for (auto it = faces.begin(); it != faces.end(); ++it) {
    faceArray[i] = it->x();
    faceArray[i + 1] = it->y();
    faceArray[i + 2] = it->z();
    i += 3;
  }

  return faceArray;
}

int vdb_float_grid::get_vertex_count() { return _vertex_count; }

int vdb_float_grid::get_face_count() { return _face_count; }

void vdb_float_grid::set(const int i, const int j, const int k,
                         const float &v) {
  typename openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();
  openvdb::Coord ijk(i, j, k);
  accessor.setValue(ijk, v);
}

float vdb_float_grid::operator()(const int i, const int j, const int k) const {
  typename openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();
  openvdb::Coord ijk(i, j, k);
  return accessor.getValue(ijk);
}

float vdb_float_grid::operator()(const float i, const float j,
                                 const float k) const {
  typename openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();
  openvdb::Coord ijk(i, j, k);
  return accessor.getValue(ijk);
}

double vdb_float_grid::calc_positive_density() const {
  double density = 0;

  vdb_sum_pos_density(*_grid, density);

  int numvoxel = _grid->activeVoxelCount();
  if (numvoxel)
    density /= numvoxel;

  openvdb::Vec3d zero(0, 0, 0);
  density *= get_volume();

  return density;
}

double vdb_float_grid::get_volume() const {
  double volume = 0;
  vdb_calc_volume(*_grid, volume);
  return volume;
}

double vdb_float_grid::get_area() const {
  double area = 0;
  vdb_calc_area(*_grid, area);
  return area;
}

int64_t vdb_float_grid::get_memory_size() const { return _grid->memUsage(); }

bool vdb_float_grid::is_sdf() const {
  if (_grid->getGridClass() == openvdb::GRID_LEVEL_SET)
    return true;

  return false;
}

std::shared_ptr<vdb_float_grid>
vdb_float_grid::resample(float_grid::Ptr refGrid, float voxelSize, int curOrder,
                         float tolerance, bool prune) {
  if (refGrid == nullptr) {
    refGrid = openvdb::FloatGrid::create();
    refGrid->setTransform(
        openvdb::math::Transform::createLinearTransform(voxelSize));
  }
  openvdb::math::Transform::Ptr refXform = refGrid->transform().copy();
  vdb_grid_ptr outGrid = _grid->copyGridWithNewTree();
  outGrid->setTransform(refXform);

  if (curOrder == 0) {
    grid_resample_to_match_op<openvdb::tools::PointSampler> op(outGrid);
    _grid->apply<all_grid_types>(op);
  } else if (curOrder == 1) {
    grid_resample_to_match_op<openvdb::tools::BoxSampler> op(outGrid);
    _grid->apply<all_grid_types>(op);
  } else if (curOrder == 2) {
    grid_resample_to_match_op<openvdb::tools::QuadraticSampler> op(outGrid);
    _grid->apply<all_grid_types>(op);
  }

  if (prune)
    outGrid->pruneGrid(tolerance);

  return std::make_shared<vdb_float_grid>(vdb_grid_cast<float_grid>(outGrid));
}

void vdb_float_grid::sparse_fill(const vdb::CoordBBox &box, const float &v,
                                 bool active) {
  _grid->sparseFill(box, v, active);
}

void vdb_float_grid::fill(const vdb::CoordBBox &box, const float &v,
                          bool active) {
  _grid->fill(box, v, active);
}

void vdb_float_grid::dense_fill(const vdb::CoordBBox &box, const float &v,
                                bool active) {
  _grid->denseFill(box, v, active);
}

} // namespace flywave
