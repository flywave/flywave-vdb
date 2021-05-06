#include "vdb_api.h"
#include "grid_api_impl.hh"

#include "float_grid.hh"
#include "mesh_data.hh"
#include "particle.hh"

#include <openvdb/tools/Dense.h>
#include <openvdb/util/Util.h>

#include <vector>

using namespace flywave;

#ifdef __cplusplus
extern "C" {
#endif

FLYWAVE_VDB_API vdb_float_grid_t *vdb_float_grid_create() {
  vdb_float_grid *grid = new vdb_float_grid();
  vdb_float_grid_t *ret = new vdb_float_grid_t{grid};
  return ret;
}

FLYWAVE_VDB_API void vdb_float_grid_free(vdb_float_grid_t *grid) {
  if (grid != nullptr) {
    delete grid->ptr;
    grid->ptr = nullptr;
    delete grid;
  }
}

FLYWAVE_VDB_API vdb_float_grid_t *
vdb_float_grid_duplicate(vdb_float_grid_t *grid) {
  vdb_float_grid *dup = new vdb_float_grid(grid->ptr);
  vdb_float_grid_t *ret = new vdb_float_grid_t{dup};
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
    openvdb::Vec4I face(vFaces[i], vFaces[i + 1], vFaces[i + 2],
                        openvdb::util::INVALID_IDX);

    vMesh.add_face(face);
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

FLYWAVE_VDB_API _Bool vdb_float_grid_transform(vdb_float_grid_t *grid,
                                               double *matrix, int mCount) {
  if (mCount != 16) {
    return false;
  }

  vdb::math::Mat4d xform = vdb::math::Mat4d(
      matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5],
      matrix[6], matrix[7], matrix[8], matrix[9], matrix[10], matrix[11],
      matrix[12], matrix[13], matrix[14], matrix[15]);

  grid->ptr->transform(xform);

  return true;
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

FLYWAVE_VDB_API vdb_pixel_grid_t *vdb_pixel_grid_create() {
  vdb_pixel_grid *grid = new vdb_pixel_grid();
  vdb_pixel_grid_t *ret = new vdb_pixel_grid_t{grid};
  return ret;
}

FLYWAVE_VDB_API void vdb_pixel_grid_free(vdb_pixel_grid_t *grid) {
  if (grid != nullptr) {
    delete grid->ptr;
    grid->ptr = nullptr;
    delete grid;
  }
}

FLYWAVE_VDB_API vdb_pixel_grid_t *
vdb_pixel_grid_duplicate(vdb_pixel_grid_t *grid) {
  vdb_pixel_grid *dup = new vdb_pixel_grid(grid->ptr);
  vdb_pixel_grid_t *ret = new vdb_pixel_grid_t{dup};
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

FLYWAVE_VDB_API _Bool vdb_pixel_grid_transform(vdb_pixel_grid_t *grid,
                                               double *matrix, int mCount) {
  if (mCount != 16) {
    return false;
  }

  vdb::math::Mat4d xform = vdb::math::Mat4d(
      matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5],
      matrix[6], matrix[7], matrix[8], matrix[9], matrix[10], matrix[11],
      matrix[12], matrix[13], matrix[14], matrix[15]);

  grid->ptr->transform(xform);

  return true;
}

#ifdef __cplusplus
}
#endif
