#include "grid_api.h"
#include "grid_api_impl.hh"
#include "voxelizer_api_impl.hh"

#include "float_grid.hh"
#include "mesh_data.hh"
#include "particle.hh"

#include <openvdb/tools/Dense.h>
#include <openvdb/util/Util.h>

#include <sstream>
#include <vector>

using namespace flywave;

#ifdef __cplusplus
extern "C" {
#endif

FLYWAVE_VDB_API vdb_float_grid_t *vdb_float_grid_create() {
  vdb_float_grid_t *ret =
      new vdb_float_grid_t{std::make_shared<vdb_float_grid>()};
  return ret;
}

FLYWAVE_VDB_API void vdb_float_grid_free(vdb_float_grid_t *grid) {
  if (grid != nullptr) {
    delete grid;
  }
}

FLYWAVE_VDB_API vdb_float_grid_t *
vdb_float_grid_duplicate(vdb_float_grid_t *grid) {
  vdb_float_grid_t *ret =
      new vdb_float_grid_t{std::make_shared<vdb_float_grid>(grid->ptr.get())};
  return ret;
}

FLYWAVE_VDB_API _Bool vdb_float_grid_read(vdb_float_grid_t *grid,
                                          const char *filename) {
  return grid->ptr->read(filename);
}

FLYWAVE_VDB_API _Bool vdb_float_grid_write(vdb_float_grid_t *grid,
                                           const char *filename) {
  return grid->ptr->write(filename);
}

FLYWAVE_VDB_API _Bool vdb_float_grid_from_points(vdb_float_grid_t *grid,
                                                 double *vPoints, int pCount,
                                                 double *vRadius, int rCount,
                                                 double voxelSize,
                                                 double bandwidth) {
  std::vector<openvdb::Vec3R> particleList;

  int i = 0;
  while (i < pCount) {
    double x = vPoints[i];
    double y = vPoints[i + 1];
    double z = vPoints[i + 2];

    particleList.push_back(openvdb::Vec3R(x, y, z));

    i += 3;
  }

  vdb_particle ps;
  ps.clear();

  if (particleList.size() == rCount) {
    int i = 0;
    for (auto it = particleList.begin(); it != particleList.end(); ++it) {
      ps.add((*it), openvdb::Real(vRadius[i]));
      i++;
    }
  } else {
    double average = 0.0;
    for (int i = 0; i < rCount; i++) {
      average += vRadius[i];
    }
    average /= rCount;
    openvdb::Real radius = openvdb::Real(average);

    for (auto it = particleList.begin(); it != particleList.end(); ++it) {
      ps.add((*it), radius);
    }
  }

  return grid->ptr->create_from_points(ps, voxelSize, bandwidth);
}

FLYWAVE_VDB_API _Bool vdb_float_grid_from_mesh(vdb_float_grid_t *grid,
                                               float *vPoints, int vCount,
                                               int *vFaces, int fCount,
                                               double voxelSize,
                                               double bandwidth) {
  double inverseVoxelSize = 1.0 / voxelSize;

  mesh_data vMesh;
  vMesh.clear();

  int i = 0;
  while (i < vCount) {

    openvdb::Vec3s vertex(vPoints[i], vPoints[i + 1], vPoints[i + 2]);

    vertex *= inverseVoxelSize;

    vMesh.add_vertice(vertex);

    i += 3;
  }

  i = 0;
  while (i < fCount) {
    openvdb::Vec3I face(vFaces[i], vFaces[i + 1], vFaces[i + 2]);

    vMesh.add_mtl_face(none_mtl_id, face);
    i += 3;
  }

  return grid->ptr->create_from_mesh(vMesh, voxelSize, bandwidth);
}

FLYWAVE_VDB_API void vdb_float_grid_to_mesh(vdb_float_grid_t *grid) {
  grid->ptr->update_display();
}

FLYWAVE_VDB_API void vdb_float_grid_to_mesh_settings(vdb_float_grid_t *grid,
                                                     double isovalue,
                                                     double adaptivity) {
  grid->ptr->update_display(isovalue, adaptivity);
}

FLYWAVE_VDB_API _Bool vdb_float_grid_empty(vdb_float_grid_t *grid) {
  return grid->ptr->grid()->empty();
}

FLYWAVE_VDB_API void vdb_float_grid_clear(vdb_float_grid_t *grid) {
  grid->ptr->grid()->clear();
}

FLYWAVE_VDB_API void vdb_float_grid_prune(vdb_float_grid_t *grid,
                                          float tolerance) {
  grid->ptr->grid()->pruneGrid(tolerance);
}

FLYWAVE_VDB_API void vdb_float_grid_clip(vdb_float_grid_t *grid, double *bbox) {
  auto cbox = vdb::BBoxd(vdb::Vec3d{bbox[0], bbox[1], bbox[2]},
                         vdb::Vec3d{bbox[3], bbox[4], bbox[5]});
  grid->ptr->grid()->clipGrid(cbox);
}

FLYWAVE_VDB_API void vdb_float_grid_clip_from_coordbox(vdb_float_grid_t *grid,
                                                       int32_t *cbox) {
  auto ccbox = vdb::CoordBBox(vdb::Coord{cbox[0], cbox[1], cbox[2]},
                              vdb::Coord{cbox[3], cbox[4], cbox[5]});
  grid->ptr->grid()->clip(ccbox);
}

FLYWAVE_VDB_API void
vdb_float_grid_active_voxel_bounding_box(vdb_float_grid_t *grid, int32_t *box) {
  auto bbox = grid->ptr->grid()->evalActiveVoxelBoundingBox();
  box[0] = bbox.min().x();
  box[1] = bbox.min().y();
  box[2] = bbox.min().z();

  box[3] = bbox.max().x();
  box[4] = bbox.max().y();
  box[5] = bbox.max().z();
}

FLYWAVE_VDB_API void vdb_float_grid_active_voxel_dim(vdb_float_grid_t *grid,
                                                     int32_t *dims) {
  auto dim = grid->ptr->grid()->evalActiveVoxelDim();
  dims[0] = dim.x();
  dims[1] = dim.y();
  dims[2] = dim.z();
}

FLYWAVE_VDB_API float *vdb_float_grid_vertex_buffer(vdb_float_grid_t *grid,
                                                    int *size) {
  float *verticeArray = grid->ptr->get_mesh_vertices();
  *size = grid->ptr->get_vertex_count();
  return verticeArray;
}

FLYWAVE_VDB_API int *vdb_float_grid_face_buffer(vdb_float_grid_t *grid,
                                                int *size) {
  int *faceArray = grid->ptr->get_mesh_faces();
  *size = grid->ptr->get_face_count();
  return faceArray;
}

FLYWAVE_VDB_API voxel_transform_t *
vdb_float_grid_get_transform(vdb_float_grid_t *grid) {
  return new voxel_transform_t{grid->ptr->grid()->transformPtr()};
}

FLYWAVE_VDB_API void vdb_float_grid_set_transform(vdb_float_grid_t *grid,
                                                  voxel_transform_t *tran) {
  grid->ptr->grid()->setTransform(tran->ptr);
}

FLYWAVE_VDB_API _Bool
vdb_float_grid_save_float_as_half(vdb_float_grid_t *grid) {
  return grid->ptr->grid()->saveFloatAsHalf();
}

FLYWAVE_VDB_API void
vdb_float_grid_set_save_float_as_half(vdb_float_grid_t *grid, _Bool v) {
  grid->ptr->grid()->setSaveFloatAsHalf(v);
}

FLYWAVE_VDB_API uint64_t
vdb_float_grid_active_voxel_count(vdb_float_grid_t *grid) {
  return grid->ptr->grid()->activeVoxelCount();
}

FLYWAVE_VDB_API void vdb_float_grid_union(vdb_float_grid_t *grid,
                                          vdb_float_grid_t *csgGrid) {
  grid->ptr->boolean_union(*csgGrid->ptr);
}

FLYWAVE_VDB_API void vdb_float_grid_difference(vdb_float_grid_t *grid,
                                               vdb_float_grid_t *csgGrid) {
  grid->ptr->boolean_difference(*csgGrid->ptr);
}

FLYWAVE_VDB_API void vdb_float_grid_intersection(vdb_float_grid_t *grid,
                                                 vdb_float_grid_t *csgGrid) {
  grid->ptr->boolean_intersection(*csgGrid->ptr);
}

FLYWAVE_VDB_API void vdb_float_grid_offset(vdb_float_grid_t *grid,
                                           double amount) {
  grid->ptr->offset(amount);
}

FLYWAVE_VDB_API void vdb_float_grid_offset_mask(vdb_float_grid_t *grid,
                                                double amount,
                                                vdb_float_grid_t *mask,
                                                double min, double max,
                                                _Bool invert) {
  grid->ptr->offset(amount, *mask->ptr, min, max, invert);
}

FLYWAVE_VDB_API void vdb_float_grid_smooth(vdb_float_grid_t *grid, int type,
                                           int iterations, int width) {
  grid->ptr->smooth(type, iterations, width);
}

FLYWAVE_VDB_API void
vdb_float_grid_smooth_mask(vdb_float_grid_t *grid, int type, int iterations,
                           int width, vdb_float_grid_t *mask, double min,
                           double max, _Bool invert) {
  grid->ptr->smooth(type, iterations, width, *mask->ptr, min, max, invert);
}

FLYWAVE_VDB_API void vdb_float_grid_blend(vdb_float_grid_t *bGrid,
                                          vdb_float_grid_t *eGrid,
                                          double bPosition, double bEnd) {
  bGrid->ptr->blend(*eGrid->ptr, bPosition, bEnd);
}

FLYWAVE_VDB_API void
vdb_float_grid_blend_mask(vdb_float_grid_t *bGrid, vdb_float_grid_t *eGrid,
                          double bPosition, double bEnd, vdb_float_grid_t *mask,
                          double min, double max, _Bool invert) {
  bGrid->ptr->blend(*eGrid->ptr, bPosition, bEnd, *mask->ptr, min, max, invert);
}

FLYWAVE_VDB_API void vdb_float_grid_rebuild(vdb_float_grid_t *bGrid, float iso,
                                            float exWidth, float inWidth) {
  bGrid->ptr->rebuild(iso, exWidth, inWidth);
}

FLYWAVE_VDB_API float *vdb_float_grid_dense(vdb_float_grid_t *bGrid, int *width,
                                            int *height, int *depth) {
  openvdb::FloatGrid::Ptr grid =
      openvdb::gridPtrCast<openvdb::FloatGrid>(bGrid->ptr->grid());

  openvdb::CoordBBox bb = grid->evalActiveVoxelBoundingBox();
  *width = abs(bb.min().x()) + abs(bb.max().x());
  *height = abs(bb.min().y()) + abs(bb.max().y());
  *depth = abs(bb.min().z()) + abs(bb.max().z());

  openvdb::Coord dim(*width, *height, *depth);
  openvdb::Coord originvdb(-abs(bb.min().x()), -abs(bb.min().y()),
                           -abs(bb.min().z()));
  openvdb::tools::Dense<float> dense(dim, originvdb);

  openvdb::tools::copyToDense<openvdb::tools::Dense<float>, openvdb::FloatGrid>(
      *grid, dense);

  int size = *width * *height * *depth;

  float *floatData = reinterpret_cast<float *>(malloc(size));

  std::copy(dense.data(), (dense.data() + size), floatData);

  return floatData;
}

FLYWAVE_VDB_API float *vdb_float_grid_closest_point(vdb_float_grid_t *grid,
                                                    float *vPoints, int vCount,
                                                    int *rSize) {
  std::vector<openvdb::Vec3R> points;
  std::vector<float> distances;

  int i = 0;
  while (i < vCount) {

    openvdb::Vec3R vertex(vPoints[i], vPoints[i + 1], vPoints[i + 2]);

    points.push_back(vertex);

    i += 3;
  }

  grid->ptr->closest_point(points, distances);

  *rSize = points.size() * 3;

  float *pArray = reinterpret_cast<float *>(malloc(*rSize * sizeof(float)));

  i = 0;
  for (auto it = points.begin(); it != points.end(); ++it) {
    pArray[i] = it->x();
    pArray[i + 1] = it->y();
    pArray[i + 2] = it->z();
    i += 3;
  }

  return pArray;
}

FLYWAVE_VDB_API void vdb_float_grid_set(vdb_float_grid_t *grid, int x, int y,
                                        int z, float v) {
  grid->ptr->set(x, y, z, v);
}

FLYWAVE_VDB_API float vdb_float_grid_get(vdb_float_grid_t *grid, int x, int y,
                                         int z) {

  return (*grid->ptr)(x, y, z);
}

FLYWAVE_VDB_API float vdb_float_grid_linear_get(vdb_float_grid_t *grid, float x,
                                                float y, float z) {
  return (*grid->ptr)(x, y, z);
}

extern FLYWAVE_VDB_API void
vdb_float_grid_eval_active_bounding_box(vdb_float_grid_t *grid, double *box) {
  auto vbox = grid->ptr->grid()->evalActiveVoxelBoundingBox();
  box[0] = vbox.min().x();
  box[1] = vbox.min().y();
  box[2] = vbox.min().z();

  box[3] = vbox.max().x();
  box[4] = vbox.max().y();
  box[5] = vbox.max().z();
}

FLYWAVE_VDB_API vdb_float_grid_t *
vdb_float_grid_resample_with_ref(vdb_float_grid_t *grid,
                                 vdb_float_grid_t *ref_grid, float voxelSize,
                                 int curOrder, float tolerance, _Bool prune) {
  return new vdb_float_grid_t{grid->ptr->resample(
      ref_grid->ptr->grid(), voxelSize, curOrder, tolerance, prune)};
}

FLYWAVE_VDB_API vdb_float_grid_t *vdb_float_grid_resample_with_grid_transform(
    vdb_float_grid_t *grid, voxel_grid_transform_t *tran, int curOrder,
    float tolerance, _Bool prune) {
  return new vdb_float_grid_t{
      grid->ptr->resample<cgo_grid_transform>(*tran->ptr, curOrder, tolerance, prune)};
}

FLYWAVE_VDB_API vdb_pixel_grid_t *vdb_pixel_grid_create() {
  vdb_pixel_grid_t *ret =
      new vdb_pixel_grid_t{std::make_shared<vdb_pixel_grid>()};
  return ret;
}

FLYWAVE_VDB_API void vdb_pixel_grid_free(vdb_pixel_grid_t *grid) {
  if (grid != nullptr) {
    delete grid;
  }
}

FLYWAVE_VDB_API vdb_pixel_grid_t *
vdb_pixel_grid_duplicate(vdb_pixel_grid_t *grid) {
  vdb_pixel_grid_t *ret =
      new vdb_pixel_grid_t{std::make_shared<vdb_pixel_grid>(grid->ptr.get())};
  return ret;
}

FLYWAVE_VDB_API _Bool vdb_pixel_grid_read(vdb_pixel_grid_t *grid,
                                          const char *filename) {
  return grid->ptr->read(filename);
}

FLYWAVE_VDB_API _Bool vdb_pixel_grid_write(vdb_pixel_grid_t *grid,
                                           const char *filename) {
  return grid->ptr->write(filename);
}

FLYWAVE_VDB_API _Bool vdb_pixel_grid_empty(vdb_pixel_grid_t *grid) {
  return grid->ptr->grid()->empty();
}

FLYWAVE_VDB_API void vdb_pixel_grid_clear(vdb_pixel_grid_t *grid) {
  grid->ptr->grid()->clear();
}

FLYWAVE_VDB_API voxel_transform_t *
vdb_pixel_grid_get_transform(vdb_pixel_grid_t *grid) {
  return new voxel_transform_t{grid->ptr->grid()->transformPtr()};
}

FLYWAVE_VDB_API void vdb_pixel_grid_set_transform(vdb_pixel_grid_t *grid,
                                                  voxel_transform_t *tran) {
  grid->ptr->grid()->setTransform(tran->ptr);
}

FLYWAVE_VDB_API char *vdb_float_grid_print_info(vdb_float_grid_t *grid) {
  std::stringstream os;
  grid->ptr->grid()->print(os);
  char *ret = (char *)malloc(sizeof(char) * os.str().size() + 1);
  memcpy(ret, os.str().c_str(), os.str().size());
  ret[os.str().size()] = '\0';
  return ret;
}

FLYWAVE_VDB_API void vdb_pixel_grid_prune(vdb_pixel_grid_t *grid,
                                          float tolerance) {
  grid->ptr->grid()->pruneGrid(tolerance);
}

FLYWAVE_VDB_API void vdb_pixel_grid_clip(vdb_pixel_grid_t *grid, double *bbox) {
  auto cbox = vdb::BBoxd(vdb::Vec3d{bbox[0], bbox[1], bbox[2]},
                         vdb::Vec3d{bbox[3], bbox[4], bbox[5]});
  grid->ptr->grid()->clipGrid(cbox);
}

FLYWAVE_VDB_API void vdb_pixel_grid_clip_from_coordbox(vdb_pixel_grid_t *grid,
                                                       int32_t *cbox) {
  auto ccbox = vdb::CoordBBox(vdb::Coord{cbox[0], cbox[1], cbox[2]},
                              vdb::Coord{cbox[3], cbox[4], cbox[5]});
  grid->ptr->grid()->clip(ccbox);
}

FLYWAVE_VDB_API void
vdb_pixel_grid_active_voxel_bounding_box(vdb_pixel_grid_t *grid, int32_t *box) {
  auto bbox = grid->ptr->grid()->evalActiveVoxelBoundingBox();
  box[0] = bbox.min().x();
  box[1] = bbox.min().y();
  box[2] = bbox.min().z();

  box[3] = bbox.max().x();
  box[4] = bbox.max().y();
  box[5] = bbox.max().z();
}

FLYWAVE_VDB_API void vdb_pixel_grid_active_voxel_dim(vdb_pixel_grid_t *grid,
                                                     int32_t *dims) {
  auto dim = grid->ptr->grid()->evalActiveVoxelDim();
  dims[0] = dim.x();
  dims[1] = dim.y();
  dims[2] = dim.z();
}

FLYWAVE_VDB_API uint64_t
vdb_pixel_grid_active_voxel_count(vdb_pixel_grid_t *grid) {
  return grid->ptr->grid()->activeVoxelCount();
}

FLYWAVE_VDB_API char *vdb_pixel_grid_print_info(vdb_pixel_grid_t *grid) {
  std::stringstream os;
  grid->ptr->grid()->print(os);
  char *ret = (char *)malloc(sizeof(char) * os.str().size() + 1);
  memcpy(ret, os.str().c_str(), os.str().size());
  ret[os.str().size()] = '\0';
  return ret;
}

FLYWAVE_VDB_API void vdb_pixel_grid_set(vdb_pixel_grid_t *grid, int x, int y,
                                        int z, vdb_pixel_t v) {
  flywave::pixel_data pd;
  pd._type = static_cast<flywave::pixel_data::type_t>(v.tp);
  pd._material_id = v.material_id;
  pd._feature_id = v.feature_id;
  pd._color.x() = v.color_r;
  pd._color.y() = v.color_g;
  pd._color.z() = v.color_b;
  pd._color.w() = v.color_a;

  grid->ptr->set(x, y, z, flywave::pixel(pd));
}

FLYWAVE_VDB_API vdb_pixel_t vdb_pixel_grid_get(vdb_pixel_grid_t *grid, int x,
                                               int y, int z) {
  flywave::pixel p = (*grid->ptr)(x, y, z);
  vdb_pixel_t ret;
  ret.tp = static_cast<vdb_pixel_type_t>(p._data._type);
  ret.material_id = p._data._material_id;
  ret.feature_id = p._data._feature_id;
  ret.color_r = p._data._color.x();
  ret.color_g = p._data._color.y();
  ret.color_b = p._data._color.z();
  ret.color_a = p._data._color.w();
  return ret;
}

FLYWAVE_VDB_API vdb_pixel_t vdb_pixel_grid_linear_get(vdb_pixel_grid_t *grid,
                                                      float x, float y,
                                                      float z) {
  flywave::pixel p = (*grid->ptr)(x, y, z);
  vdb_pixel_t ret;
  ret.tp = static_cast<vdb_pixel_type_t>(p._data._type);
  ret.material_id = p._data._material_id;
  ret.feature_id = p._data._feature_id;
  ret.color_r = p._data._color.x();
  ret.color_g = p._data._color.y();
  ret.color_b = p._data._color.z();
  ret.color_a = p._data._color.w();
  return ret;
}
#ifdef __cplusplus
}
#endif
