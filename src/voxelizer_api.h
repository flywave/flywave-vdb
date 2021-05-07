#pragma once

#include "grid_api.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(WIN32) || defined(WINDOWS) || defined(_WIN32) || defined(_WINDOWS)
#define FLYWAVE_VDB_API __declspec(dllexport)
#else
#define FLYWAVE_VDB_API
#endif

typedef struct _voxel_pixel_t voxel_pixel_t;
typedef struct _voxel_texture_atlas_generator_t texture_atlas_generator_t;
typedef struct _voxel_mesh_t voxel_mesh_t;
typedef struct _voxel_texture_t voxel_texture_t;
typedef struct _voxel_material_t voxel_material_t;
typedef struct _voxel_pixel_material_data_t voxel_pixel_material_data_t;
typedef struct _voxel_pixel_materials_t voxel_pixel_materials_t;
typedef struct _voxel_pixel_texture_data_t voxel_pixel_texture_data_t;
typedef struct _voxel_pixel_mesh_data_t voxel_pixel_mesh_data_t;
typedef struct _voxel_mesh_builder_t voxel_mesh_builder_t;
typedef struct _voxel_border_lock_t voxel_border_lock_t;
typedef struct _voxel_filter_triangle_t voxel_filter_triangle_t;
typedef struct _voxel_transform_t voxel_transform_t;
typedef struct _voxel_texture_mesh_t voxel_texture_mesh_t;

extern FLYWAVE_VDB_API voxel_pixel_t *voxel_pixel_create();
extern FLYWAVE_VDB_API void voxel_pixel_free(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API voxel_pixel_t *voxel_pixel_duplicate(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API _Bool voxel_pixel_read(voxel_pixel_t *vox,
                                              const char *path);
extern FLYWAVE_VDB_API _Bool voxel_pixel_write(voxel_pixel_t *vox,
                                               const char *path);
extern FLYWAVE_VDB_API void
voxel_pixel_composite(voxel_pixel_t *vox, voxel_pixel_t *tg, uint32_t type);
extern FLYWAVE_VDB_API void voxel_pixel_clear(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API _Bool voxel_pixel_ray_test(voxel_pixel_t *vox,
                                                  double *ray_origin,
                                                  double *ray_direction,
                                                  double *p);
extern FLYWAVE_VDB_API voxel_transform_t *
voxel_pixel_voxel_resolution(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API vdb_float_grid_t *
voxel_pixel_get_voxel_grid(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API vdb_pixel_grid_t *
voxel_pixel_get_pixel_grid(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API void voxel_pixel_set_voxel_grid(voxel_pixel_t *vox,
                                                       vdb_float_grid_t *vg);
extern FLYWAVE_VDB_API void voxel_pixel_set_pixel_grid(voxel_pixel_t *vox,
                                                       vdb_pixel_grid_t *vg);
extern FLYWAVE_VDB_API _Bool voxel_pixel_is_empty(voxel_pixel_t *vox);

extern FLYWAVE_VDB_API voxel_pixel_materials_t *
voxel_pixel_get_materials(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API void
voxel_pixel_set_materials(voxel_pixel_t *vox, voxel_pixel_materials_t *mtls);

extern FLYWAVE_VDB_API vdb_pixel_grid_t *
voxel_pixel_extract_color(voxel_pixel_t *vox, voxel_pixel_t *svox);

extern FLYWAVE_VDB_API void voxel_pixel_fill_color(voxel_pixel_t *vox,
                                                   voxel_pixel_t *svox,
                                                   vdb_pixel_grid_t *colors);

extern FLYWAVE_VDB_API void
voxel_pixel_eval_max_min_elevation(voxel_pixel_t *vox, double *bboxin,
                                   double *bboxout);

typedef struct _voxel_io_vertex_t {
  float v_x;
  float v_y;
  float v_z;
  uint8_t c_r;
  uint8_t c_g;
  uint8_t c_b;
  uint8_t c_a;
  float t_u;
  float t_v;
  _Bool b;
} voxel_io_vertex_t;

typedef struct _voxel_io_triangle_t {
  voxel_io_vertex_t v_a;
  voxel_io_vertex_t v_b;
  voxel_io_vertex_t v_c;
  uint32_t node;
  uint32_t tex;
  uint32_t mtl;
  uint64_t feature_id;
} voxel_io_triangle_t;

extern FLYWAVE_VDB_API void
voxel_pixel_make_triangles(voxel_pixel_t *vox, voxel_io_triangle_t **tris,
                           size_t *tricount, double *mat, size_t tex_offset,
                           size_t mtl_offset, voxel_border_lock_t *bl,
                           voxel_filter_triangle_t *ftri, double fquality,
                           double isovalue, double adapter);

extern FLYWAVE_VDB_API void voxel_pixel_make_triangles_simple(
    voxel_pixel_t *vox, voxel_io_triangle_t **tris, size_t *tricount,
    double *mat, size_t tex_offset, size_t mtl_offset, double fquality,
    double isovalue, double adapter);

extern FLYWAVE_VDB_API texture_atlas_generator_t *
voxel_pixel_make_texture_atlas_generator(voxel_pixel_t *vox,
                                         voxel_texture_mesh_t *tmesh,
                                         double *mat, float pixel_pad);

#ifdef __cplusplus
}
#endif
