#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(WIN32) || defined(WINDOWS) || defined(_WIN32) || defined(_WINDOWS)
#define FLYWAVE_VDB_API __declspec(dllexport)
#else
#define FLYWAVE_VDB_API
#endif

typedef struct _vdb_float_grid_t vdb_float_grid_t;

extern FLYWAVE_VDB_API vdb_float_grid_t *vdb_float_grid_create();
extern FLYWAVE_VDB_API void vdb_float_grid_free(vdb_float_grid_t *grid);
extern FLYWAVE_VDB_API vdb_float_grid_t *
vdb_float_grid_duplicate(vdb_float_grid_t *grid);

extern FLYWAVE_VDB_API _Bool vdb_float_grid_read(vdb_float_grid_t *grid,
                                                 const char *filename);
extern FLYWAVE_VDB_API _Bool vdb_float_grid_write(vdb_float_grid_t *grid,
                                                  const char *filename);

extern FLYWAVE_VDB_API _Bool vdb_float_grid_from_points(
    vdb_float_grid_t *grid, double *vPoints, int pCount, double *vRadius,
    int rCount, double voxelSize, double bandwidth);
extern FLYWAVE_VDB_API _Bool vdb_float_grid_from_mesh(
    vdb_float_grid_t *grid, float *vPoints, int vCount, int *vFaces, int fCount,
    double voxelSize, double bandwidth);

extern FLYWAVE_VDB_API void vdb_float_grid_to_mesh(vdb_float_grid_t *grid);
extern FLYWAVE_VDB_API void
vdb_float_grid_to_mesh_settings(vdb_float_grid_t *grid, double isovalue,
                                double adaptivity);

extern FLYWAVE_VDB_API float *
vdb_float_grid_vertex_buffer(vdb_float_grid_t *grid, int *size);
extern FLYWAVE_VDB_API int *vdb_float_grid_face_buffer(vdb_float_grid_t *grid,
                                                       int *size);

extern FLYWAVE_VDB_API _Bool vdb_float_grid_transform(vdb_float_grid_t *grid,
                                                      double *matrix,
                                                      int mCount);

extern FLYWAVE_VDB_API void vdb_float_grid_union(vdb_float_grid_t *grid,
                                                 vdb_float_grid_t *csgGrid);
extern FLYWAVE_VDB_API void
vdb_float_grid_difference(vdb_float_grid_t *grid, vdb_float_grid_t *csgGrid);
extern FLYWAVE_VDB_API void
vdb_float_grid_intersection(vdb_float_grid_t *grid, vdb_float_grid_t *csgGrid);

extern FLYWAVE_VDB_API void vdb_float_grid_offset(vdb_float_grid_t *grid,
                                                  double amount);
extern FLYWAVE_VDB_API void vdb_float_grid_offset_mask(vdb_float_grid_t *grid,
                                                       double amount,
                                                       vdb_float_grid_t *mask,
                                                       double min, double max,
                                                       _Bool invert);
extern FLYWAVE_VDB_API void vdb_float_grid_smooth(vdb_float_grid_t *grid,
                                                  int type, int iterations,
                                                  int width);
extern FLYWAVE_VDB_API void
vdb_float_grid_smooth_mask(vdb_float_grid_t *grid, int type, int iterations,
                           int width, vdb_float_grid_t *mask, double min,
                           double max, _Bool invert);
extern FLYWAVE_VDB_API void vdb_float_grid_blend(vdb_float_grid_t *bGrid,
                                                 vdb_float_grid_t *eGrid,
                                                 double bPosition, double bEnd);
extern FLYWAVE_VDB_API void
vdb_float_grid_blend_mask(vdb_float_grid_t *bGrid, vdb_float_grid_t *eGrid,
                          double bPosition, double bEnd, vdb_float_grid_t *mask,
                          double min, double max, _Bool invert);
extern FLYWAVE_VDB_API void vdb_float_grid_rebuild(vdb_float_grid_t *bGrid,
                                                   float iso, float exWidth,
                                                   float inWidth);
extern FLYWAVE_VDB_API float *vdb_float_grid_dense(vdb_float_grid_t *bGrid,
                                                   int *width, int *height,
                                                   int *depth);

extern FLYWAVE_VDB_API float *vdb_closest_point(vdb_float_grid_t *grid,
                                                float *vPoints, int vCount,
                                                int *rSize);

extern FLYWAVE_VDB_API void vdb_float_grid_set(vdb_float_grid_t *grid, int x,
                                               int y, int z, float v);

extern FLYWAVE_VDB_API float vdb_float_grid_get(vdb_float_grid_t *grid, int x,
                                                int y, int z);

extern FLYWAVE_VDB_API float
vdb_float_grid_linear_get(vdb_float_grid_t *grid, float x, float y, float z);

typedef struct _vdb_pixel_grid_t vdb_pixel_grid_t;

extern FLYWAVE_VDB_API vdb_pixel_grid_t *vdb_pixel_grid_create();
extern FLYWAVE_VDB_API void vdb_pixel_grid_free(vdb_pixel_grid_t *grid);
extern FLYWAVE_VDB_API vdb_pixel_grid_t *
vdb_pixel_grid_duplicate(vdb_pixel_grid_t *grid);

extern FLYWAVE_VDB_API _Bool vdb_pixel_grid_read(vdb_pixel_grid_t *grid,
                                                 const char *filename);
extern FLYWAVE_VDB_API _Bool vdb_pixel_grid_write(vdb_pixel_grid_t *grid,
                                                  const char *filename);

extern FLYWAVE_VDB_API _Bool vdb_pixel_grid_transform(vdb_pixel_grid_t *grid,
                                                      double *matrix,
                                                      int mCount);
enum vdb_pixel_type_t : uint8_t {
  color,
  material,
  material_and_color,
  invalid
};

typedef struct _vdb_pixel_t {
  vdb_pixel_type_t type;
  uint8_t material_id;
  uint16_t feature_id;
  uint8_t color_r;
  uint8_t color_g;
  uint8_t color_b;
  uint8_t color_a;
} vdb_pixel_t;

extern FLYWAVE_VDB_API void vdb_pixel_grid_set(vdb_pixel_grid_t *grid, int x,
                                               int y, int z, vdb_pixel_t v);

extern FLYWAVE_VDB_API vdb_pixel_t vdb_pixel_grid_get(vdb_pixel_grid_t *grid,
                                                      int x, int y, int z);

extern FLYWAVE_VDB_API vdb_pixel_t
vdb_pixel_grid_linear_get(vdb_pixel_grid_t *grid, float x, float y, float z);

#ifdef __cplusplus
}
#endif
