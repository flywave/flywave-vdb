#pragma once

#include "vdb_api.h"
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

typedef struct _voxel_pixel_t voxel_pixel_t;
typedef struct _io_vertex_t io_vertex_t;
typedef struct _io_triangle_t io_triangle_t;
typedef struct _textute_repacker_t textute_repacker_t;
typedef struct _voxel_mesh_t voxel_mesh_t;
typedef struct _voxel_texture_t voxel_texture_t;
typedef struct _voxel_material_t voxel_material_t;
typedef struct _voxel_pixel_material_data_t voxel_pixel_material_data_t;
typedef struct _voxel_pixel_materials_t voxel_pixel_materials_t;
typedef struct _voxel_pixel_texture_data_t voxel_pixel_texture_data_t;
typedef struct _voxel_pixel_mesh_data_t voxel_pixel_mesh_data_t;
typedef struct _voxel_mesh_builder_t voxel_mesh_builder_t;

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
extern FLYWAVE_VDB_API void voxel_pixel_voxel_resolution(voxel_pixel_t *vox,
                                                         double *mat);
extern FLYWAVE_VDB_API vdb_float_grid_t *
voxel_pixel_get_voxel_grid(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API vdb_float_grid_t *
voxel_pixel_get_pixel_grid(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API void voxel_pixel_set_voxel_grid(voxel_pixel_t *vox,
                                                       vdb_float_grid_t *vg);
extern FLYWAVE_VDB_API void voxel_pixel_set_pixel_grid(voxel_pixel_t *vox,
                                                       vdb_float_grid_t *vg);
extern FLYWAVE_VDB_API _Bool voxel_pixel_is_empty(voxel_pixel_t *vox);

extern FLYWAVE_VDB_API voxel_pixel_materials_t *
voxel_pixel_get_materials(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API void
voxel_pixel_set_materials(voxel_pixel_t *vox, voxel_pixel_materials_t *mtls);

extern FLYWAVE_VDB_API void make_triangles(voxel_pixel_t *vox);


#ifdef __cplusplus
}
#endif
