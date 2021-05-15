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

FLYWAVE_VDB_API float vdb_float_grid_get_background(vdb_float_grid_t *grid) {
  return grid->ptr->grid()->background();
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
  return new vdb_float_grid_t{grid->ptr->resample<cgo_grid_transform>(
      *tran->ptr, curOrder, tolerance, prune)};
}

FLYWAVE_VDB_API void vdb_float_grid_sparse_fill(vdb_float_grid_t *grid,
                                                int32_t *box, float v,
                                                _Bool active) {
  auto ccbox = vdb::CoordBBox(vdb::Coord{box[0], box[1], box[2]},
                              vdb::Coord{box[3], box[4], box[5]});
  grid->ptr->sparse_fill(ccbox, v, active);
}

FLYWAVE_VDB_API void vdb_float_grid_fill(vdb_float_grid_t *grid, int32_t *box,
                                         float v, _Bool active) {
  auto ccbox = vdb::CoordBBox(vdb::Coord{box[0], box[1], box[2]},
                              vdb::Coord{box[3], box[4], box[5]});
  grid->ptr->fill(ccbox, v, active);
}

FLYWAVE_VDB_API void vdb_float_grid_dense_fill(vdb_float_grid_t *grid,
                                               int32_t *box, float v,
                                               _Bool active) {
  auto ccbox = vdb::CoordBBox(vdb::Coord{box[0], box[1], box[2]},
                              vdb::Coord{box[3], box[4], box[5]});
  grid->ptr->dense_fill(ccbox, v, active);
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

FLYWAVE_VDB_API void vdb_pixel_grid_sparse_fill(vdb_pixel_grid_t *grid,
                                                int32_t *box, vdb_pixel_t v,
                                                _Bool active) {
  auto ccbox = vdb::CoordBBox(vdb::Coord{box[0], box[1], box[2]},
                              vdb::Coord{box[3], box[4], box[5]});
  grid->ptr->sparse_fill(ccbox, *reinterpret_cast<flywave::pixel *>(&v),
                         active);
}

FLYWAVE_VDB_API void vdb_pixel_grid_fill(vdb_pixel_grid_t *grid, int32_t *box,
                                         vdb_pixel_t v, _Bool active) {
  auto ccbox = vdb::CoordBBox(vdb::Coord{box[0], box[1], box[2]},
                              vdb::Coord{box[3], box[4], box[5]});
  grid->ptr->fill(ccbox, *reinterpret_cast<flywave::pixel *>(&v), active);
}

FLYWAVE_VDB_API void vdb_pixel_grid_dense_fill(vdb_pixel_grid_t *grid,
                                               int32_t *box, vdb_pixel_t v,
                                               _Bool active) {
  auto ccbox = vdb::CoordBBox(vdb::Coord{box[0], box[1], box[2]},
                              vdb::Coord{box[3], box[4], box[5]});
  grid->ptr->dense_fill(ccbox, *reinterpret_cast<flywave::pixel *>(&v), active);
}

extern _Bool vdbFloatGridVisiton(void *ctx, int32_t *coord, float val);
extern _Bool vdbPixelGridVisiton(void *ctx, int32_t *coord, vdb_pixel_t val);

extern _Bool vdbFloatGridVisitonIterator(void *ctx,
                                         vdb_float_grid_iterator_t *iter);
extern _Bool vdbPixelGridVisitonIterator(void *ctx,
                                         vdb_pixel_grid_iterator_t *iter);

enum VdbIteratorType { VDB_ITERATOR_ON, VDB_ITERATOR_OFF, VDB_ITERATOR_ALL };

struct _vdb_float_grid_iterator_t {
  VdbIteratorType type;
  void *iter;
};

struct _vdb_pixel_grid_iterator_t {
  VdbIteratorType type;
  void *iter;
};

FLYWAVE_VDB_API void vdb_float_grid_visit_iterator_on(vdb_float_grid_t *grid,
                                                      void *v) {
  _vdb_float_grid_iterator_t cit{VDB_ITERATOR_ON, nullptr};
  for (auto it = grid->ptr->grid()->beginValueOn(); it; ++it) {
    cit.iter = &it;
    if (!vdbFloatGridVisitonIterator(v, &cit)) {
      break;
    }
  }
}

FLYWAVE_VDB_API void vdb_float_grid_visit_iterator_off(vdb_float_grid_t *grid,
                                                       void *v) {
  _vdb_float_grid_iterator_t cit{VDB_ITERATOR_OFF, nullptr};
  for (auto it = grid->ptr->grid()->beginValueOn(); it; ++it) {
    cit.iter = &it;
    if (!vdbFloatGridVisitonIterator(v, &cit)) {
      break;
    }
  }
}

FLYWAVE_VDB_API void vdb_float_grid_visit_iterator_all(vdb_float_grid_t *grid,
                                                       void *v) {
  _vdb_float_grid_iterator_t cit{VDB_ITERATOR_ALL, nullptr};
  for (auto it = grid->ptr->grid()->beginValueOn(); it; ++it) {
    cit.iter = &it;
    if (!vdbFloatGridVisitonIterator(v, &cit)) {
      break;
    }
  }
}

FLYWAVE_VDB_API void vdb_float_grid_visit_on(vdb_float_grid_t *grid, void *v) {
  for (auto it = grid->ptr->grid()->beginValueOn(); it; ++it) {
    auto coord = it.getCoord();
    if (!vdbFloatGridVisiton(v, coord.asPointer(), it.getValue()))
      break;
  }
}

FLYWAVE_VDB_API void vdb_float_grid_visit_off(vdb_float_grid_t *grid, void *v) {
  for (auto it = grid->ptr->grid()->beginValueOff(); it; ++it) {
    auto coord = it.getCoord();
    if (!vdbFloatGridVisiton(v, coord.asPointer(), it.getValue()))
      break;
  }
}

FLYWAVE_VDB_API void vdb_float_grid_visit_all(vdb_float_grid_t *grid, void *v) {
  for (auto it = grid->ptr->grid()->beginValueAll(); it; ++it) {
    auto coord = it.getCoord();
    if (!vdbFloatGridVisiton(v, coord.asPointer(), it.getValue()))
      break;
  }
}

FLYWAVE_VDB_API void
vdb_float_grid_iterator_set_value(vdb_float_grid_iterator_t *it, float v) {
  if (it->type == VDB_ITERATOR_ON) {
    reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->setValue(v);
  } else if (it->type == VDB_ITERATOR_OFF) {
    reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->setValue(v);
  } else if (it->type == VDB_ITERATOR_ALL) {
    reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->setValue(v);
  }
}

FLYWAVE_VDB_API float
vdb_float_grid_iterator_get_value(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->getValue();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->getValue();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->getValue();
  }
  return 0.0;
}

FLYWAVE_VDB_API void
vdb_float_grid_iterator_set_active_state(vdb_float_grid_iterator_t *it,
                                         _Bool v) {
  if (it->type == VDB_ITERATOR_ON) {
    reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->setActiveState(v);
  } else if (it->type == VDB_ITERATOR_OFF) {
    reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->setActiveState(v);
  } else if (it->type == VDB_ITERATOR_ALL) {
    reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->setActiveState(v);
  }
}

FLYWAVE_VDB_API void
vdb_float_grid_iterator_set_value_off(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->setValueOff();
  } else if (it->type == VDB_ITERATOR_OFF) {
    reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->setValueOff();
  } else if (it->type == VDB_ITERATOR_ALL) {
    reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->setValueOff();
  }
}

FLYWAVE_VDB_API void
vdb_float_grid_iterator_set_min_depth(vdb_float_grid_iterator_t *it,
                                      int32_t v) {
  if (it->type == VDB_ITERATOR_ON) {
    reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->setMinDepth(v);
  } else if (it->type == VDB_ITERATOR_OFF) {
    reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->setMinDepth(v);
  } else if (it->type == VDB_ITERATOR_ALL) {
    reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->setMinDepth(v);
  }
}

FLYWAVE_VDB_API int32_t
vdb_float_grid_iterator_get_min_depth(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->getMinDepth();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)
        ->getMinDepth();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)
        ->getMinDepth();
  }
  return -1;
}

FLYWAVE_VDB_API void
vdb_float_grid_iterator_set_max_depth(vdb_float_grid_iterator_t *it,
                                      int32_t v) {
  if (it->type == VDB_ITERATOR_ON) {
    reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->setMaxDepth(v);
  } else if (it->type == VDB_ITERATOR_OFF) {
    reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->setMaxDepth(v);
  } else if (it->type == VDB_ITERATOR_ALL) {
    reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->setMaxDepth(v);
  }
}
FLYWAVE_VDB_API int32_t
vdb_float_grid_iterator_get_max_depth(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->getMaxDepth();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)
        ->getMaxDepth();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)
        ->getMaxDepth();
  }
  return -1;
}

FLYWAVE_VDB_API _Bool
vdb_float_grid_iterator_test(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->test();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->test();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->test();
  }
  return false;
}

FLYWAVE_VDB_API _Bool
vdb_float_grid_iterator_is_tile_value(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->isTileValue();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)
        ->isTileValue();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)
        ->isTileValue();
  }
  return false;
}

FLYWAVE_VDB_API _Bool
vdb_float_grid_iterator_is_voxel_value(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)
        ->isVoxelValue();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)
        ->isVoxelValue();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)
        ->isVoxelValue();
  }
  return false;
}

FLYWAVE_VDB_API _Bool
vdb_float_grid_iterator_is_value_on(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->isValueOn();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->isValueOn();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->isValueOn();
  }
  return false;
}

FLYWAVE_VDB_API int32_t
vdb_float_grid_iterator_get_level(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->getLevel();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->getLevel();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->getLevel();
  }
  return -1;
}

FLYWAVE_VDB_API int32_t
vdb_float_grid_iterator_get_depth(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->getDepth();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->getDepth();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->getDepth();
  }
  return -1;
}

FLYWAVE_VDB_API int32_t
vdb_float_grid_iterator_get_leaf_depth(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)
        ->getLeafDepth();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)
        ->getLeafDepth();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)
        ->getLeafDepth();
  }
  return -1;
}

FLYWAVE_VDB_API void
vdb_float_grid_iterator_get_coord(vdb_float_grid_iterator_t *it,
                                  int32_t *coord) {
  vdb::Coord ccoord;
  if (it->type == VDB_ITERATOR_ON) {
    ccoord = reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->getCoord();
  } else if (it->type == VDB_ITERATOR_OFF) {
    ccoord = reinterpret_cast<float_grid::ValueOffIter *>(it->iter)->getCoord();
  } else if (it->type == VDB_ITERATOR_ALL) {
    ccoord = reinterpret_cast<float_grid::ValueAllIter *>(it->iter)->getCoord();
  }
  coord[0] = ccoord.x();
  coord[1] = ccoord.y();
  coord[2] = ccoord.z();
}

FLYWAVE_VDB_API void
vdb_float_grid_iterator_get_bounding_box(vdb_float_grid_iterator_t *it,
                                         int32_t *box) {
  vdb::CoordBBox cbox;
  if (it->type == VDB_ITERATOR_ON) {
    cbox =
        reinterpret_cast<float_grid::ValueOnIter *>(it->iter)->getBoundingBox();
  } else if (it->type == VDB_ITERATOR_OFF) {
    cbox = reinterpret_cast<float_grid::ValueOffIter *>(it->iter)
               ->getBoundingBox();
  } else if (it->type == VDB_ITERATOR_ALL) {
    cbox = reinterpret_cast<float_grid::ValueAllIter *>(it->iter)
               ->getBoundingBox();
  }
  box[0] = cbox.min().x();
  box[1] = cbox.min().y();
  box[2] = cbox.min().z();

  box[3] = cbox.max().x();
  box[4] = cbox.max().y();
  box[5] = cbox.max().z();
}

FLYWAVE_VDB_API int32_t
vdb_float_grid_iterator_get_voxel_count(vdb_float_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<float_grid::ValueOnIter *>(it->iter)
        ->getVoxelCount();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<float_grid::ValueOffIter *>(it->iter)
        ->getVoxelCount();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<float_grid::ValueAllIter *>(it->iter)
        ->getVoxelCount();
  }
  return 0;
}

FLYWAVE_VDB_API void vdb_pixel_grid_visit_iterator_on(vdb_pixel_grid_t *grid,
                                                      void *v) {
  _vdb_pixel_grid_iterator_t cit{VDB_ITERATOR_ON, nullptr};
  for (auto it = grid->ptr->grid()->beginValueOn(); it; ++it) {
    cit.iter = &it;
    if (!vdbPixelGridVisitonIterator(v, &cit)) {
      break;
    }
  }
}

FLYWAVE_VDB_API void vdb_pixel_grid_visit_iterator_off(vdb_pixel_grid_t *grid,
                                                       void *v) {
  _vdb_pixel_grid_iterator_t cit{VDB_ITERATOR_OFF, nullptr};
  for (auto it = grid->ptr->grid()->beginValueOn(); it; ++it) {
    cit.iter = &it;
    if (!vdbPixelGridVisitonIterator(v, &cit)) {
      break;
    }
  }
}

FLYWAVE_VDB_API void vdb_pixel_grid_visit_iterator_all(vdb_pixel_grid_t *grid,
                                                       void *v) {
  _vdb_pixel_grid_iterator_t cit{VDB_ITERATOR_ALL, nullptr};
  for (auto it = grid->ptr->grid()->beginValueOn(); it; ++it) {
    cit.iter = &it;
    if (!vdbPixelGridVisitonIterator(v, &cit)) {
      break;
    }
  }
}

FLYWAVE_VDB_API void vdb_pixel_grid_visit_on(vdb_pixel_grid_t *grid, void *v) {
  for (auto it = grid->ptr->grid()->beginValueOn(); it; ++it) {
    auto coord = it.getCoord();
    if (!vdbPixelGridVisiton(
            v, coord.asPointer(),
            *reinterpret_cast<const vdb_pixel_t *>(&it.getValue())))
      break;
  }
}

FLYWAVE_VDB_API void vdb_pixel_grid_visit_off(vdb_pixel_grid_t *grid, void *v) {
  for (auto it = grid->ptr->grid()->beginValueOff(); it; ++it) {
    auto coord = it.getCoord();
    if (!vdbPixelGridVisiton(
            v, coord.asPointer(),
            *reinterpret_cast<const vdb_pixel_t *>(&it.getValue())))
      break;
  }
}

FLYWAVE_VDB_API void vdb_pixel_grid_visit_all(vdb_pixel_grid_t *grid, void *v) {
  for (auto it = grid->ptr->grid()->beginValueAll(); it; ++it) {
    auto coord = it.getCoord();
    if (!vdbPixelGridVisiton(
            v, coord.asPointer(),
            *reinterpret_cast<const vdb_pixel_t *>(&it.getValue())))
      break;
  }
}

FLYWAVE_VDB_API void
vdb_pixel_grid_iterator_set_value(vdb_pixel_grid_iterator_t *it,
                                  vdb_pixel_t v) {
  if (it->type == VDB_ITERATOR_ON) {
    reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->setValue(
        *reinterpret_cast<flywave::pixel *>(&v));
  } else if (it->type == VDB_ITERATOR_OFF) {
    reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->setValue(
        *reinterpret_cast<flywave::pixel *>(&v));
  } else if (it->type == VDB_ITERATOR_ALL) {
    reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->setValue(
        *reinterpret_cast<flywave::pixel *>(&v));
  }
}
FLYWAVE_VDB_API vdb_pixel_t
vdb_pixel_grid_iterator_get_value(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    auto p = reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->getValue();
    return *reinterpret_cast<vdb_pixel_t *>(&p);
  } else if (it->type == VDB_ITERATOR_OFF) {
    auto p = reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->getValue();
    return *reinterpret_cast<vdb_pixel_t *>(&p);
  } else if (it->type == VDB_ITERATOR_ALL) {
    auto p = reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->getValue();
    return *reinterpret_cast<vdb_pixel_t *>(&p);
  }
  return vdb_pixel_t{};
}

FLYWAVE_VDB_API void
vdb_pixel_grid_iterator_set_active_state(vdb_pixel_grid_iterator_t *it,
                                         _Bool v) {
  if (it->type == VDB_ITERATOR_ON) {
    reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->setActiveState(v);
  } else if (it->type == VDB_ITERATOR_OFF) {
    reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->setActiveState(v);
  } else if (it->type == VDB_ITERATOR_ALL) {
    reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->setActiveState(v);
  }
}

FLYWAVE_VDB_API void
vdb_pixel_grid_iterator_set_value_off(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->setValueOff();
  } else if (it->type == VDB_ITERATOR_OFF) {
    reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->setValueOff();
  } else if (it->type == VDB_ITERATOR_ALL) {
    reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->setValueOff();
  }
}

FLYWAVE_VDB_API void
vdb_pixel_grid_iterator_set_min_depth(vdb_pixel_grid_iterator_t *it,
                                      int32_t v) {

  if (it->type == VDB_ITERATOR_ON) {
    reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->setMinDepth(v);
  } else if (it->type == VDB_ITERATOR_OFF) {
    reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->setMinDepth(v);
  } else if (it->type == VDB_ITERATOR_ALL) {
    reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->setMinDepth(v);
  }
}
FLYWAVE_VDB_API int32_t
vdb_pixel_grid_iterator_get_min_depth(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->getMinDepth();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)
        ->getMinDepth();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)
        ->getMinDepth();
  }
  return -1;
}

FLYWAVE_VDB_API void
vdb_pixel_grid_iterator_set_max_depth(vdb_pixel_grid_iterator_t *it,
                                      int32_t v) {
  if (it->type == VDB_ITERATOR_ON) {
    reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->setMaxDepth(v);
  } else if (it->type == VDB_ITERATOR_OFF) {
    reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->setMaxDepth(v);
  } else if (it->type == VDB_ITERATOR_ALL) {
    reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->setMaxDepth(v);
  }
}

FLYWAVE_VDB_API int32_t
vdb_pixel_grid_iterator_get_max_depth(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->getMaxDepth();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)
        ->getMaxDepth();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)
        ->getMaxDepth();
  }
  return -1;
}

FLYWAVE_VDB_API _Bool
vdb_pixel_grid_iterator_test(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->test();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->test();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->test();
  }
  return false;
}

FLYWAVE_VDB_API _Bool
vdb_pixel_grid_iterator_is_tile_value(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->isTileValue();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)
        ->isTileValue();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)
        ->isTileValue();
  }
  return false;
}

FLYWAVE_VDB_API _Bool
vdb_pixel_grid_iterator_is_voxel_value(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)
        ->isVoxelValue();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)
        ->isVoxelValue();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)
        ->isVoxelValue();
  }
  return false;
}

FLYWAVE_VDB_API _Bool
vdb_pixel_grid_iterator_is_value_on(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->isValueOn();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->isValueOn();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->isValueOn();
  }
  return false;
}

FLYWAVE_VDB_API int32_t
vdb_pixel_grid_iterator_get_level(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->getLevel();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->getLevel();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->getLevel();
  }
  return -1;
}

FLYWAVE_VDB_API int32_t
vdb_pixel_grid_iterator_get_depth(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->getDepth();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->getDepth();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->getDepth();
  }
  return -1;
}

FLYWAVE_VDB_API int32_t
vdb_pixel_grid_iterator_get_leaf_depth(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)
        ->getLeafDepth();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)
        ->getLeafDepth();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)
        ->getLeafDepth();
  }
  return -1;
}

FLYWAVE_VDB_API void
vdb_pixel_grid_iterator_get_coord(vdb_pixel_grid_iterator_t *it,
                                  int32_t *coord) {
  vdb::Coord ccoord;
  if (it->type == VDB_ITERATOR_ON) {
    ccoord = reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->getCoord();
  } else if (it->type == VDB_ITERATOR_OFF) {
    ccoord = reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)->getCoord();
  } else if (it->type == VDB_ITERATOR_ALL) {
    ccoord = reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)->getCoord();
  }
  coord[0] = ccoord.x();
  coord[1] = ccoord.y();
  coord[2] = ccoord.z();
}

FLYWAVE_VDB_API void
vdb_pixel_grid_iterator_get_bounding_box(vdb_pixel_grid_iterator_t *it,
                                         int32_t *box) {
  vdb::CoordBBox cbox;
  if (it->type == VDB_ITERATOR_ON) {
    cbox =
        reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)->getBoundingBox();
  } else if (it->type == VDB_ITERATOR_OFF) {
    cbox = reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)
               ->getBoundingBox();
  } else if (it->type == VDB_ITERATOR_ALL) {
    cbox = reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)
               ->getBoundingBox();
  }
  box[0] = cbox.min().x();
  box[1] = cbox.min().y();
  box[2] = cbox.min().z();

  box[3] = cbox.max().x();
  box[4] = cbox.max().y();
  box[5] = cbox.max().z();
}

FLYWAVE_VDB_API int32_t
vdb_pixel_grid_iterator_get_voxel_count(vdb_pixel_grid_iterator_t *it) {
  if (it->type == VDB_ITERATOR_ON) {
    return reinterpret_cast<pixel_grid::ValueOnIter *>(it->iter)
        ->getVoxelCount();
  } else if (it->type == VDB_ITERATOR_OFF) {
    return reinterpret_cast<pixel_grid::ValueOffIter *>(it->iter)
        ->getVoxelCount();
  } else if (it->type == VDB_ITERATOR_ALL) {
    return reinterpret_cast<pixel_grid::ValueAllIter *>(it->iter)
        ->getVoxelCount();
  }
  return 0;
}

#ifdef __cplusplus
}
#endif
