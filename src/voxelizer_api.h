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
typedef struct _voxel_pixel_material_data_t voxel_pixel_material_data_t;
typedef struct _voxel_pixel_materials_t voxel_pixel_materials_t;
typedef struct _voxel_pixel_texture_data_t voxel_pixel_texture_data_t;
typedef struct _voxel_pixel_mesh_data_t voxel_pixel_mesh_data_t;
typedef struct _voxel_mesh_builder_t voxel_mesh_builder_t;
typedef struct _voxel_border_lock_t voxel_border_lock_t;
typedef struct _voxel_filter_triangle_t voxel_filter_triangle_t;
typedef struct _voxel_transform_t voxel_transform_t;
typedef struct _voxel_texture_mesh_t voxel_texture_mesh_t;
typedef struct _voxel_texture2d_t voxel_texture2d_t;

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
voxel_pixel_make_texture_atlas_generator(voxel_pixel_t *vox, double *mat,
                                         float pixel_pad);

extern FLYWAVE_VDB_API void
voxel_texture_atlas_generator_free(texture_atlas_generator_t *tag);

extern FLYWAVE_VDB_API void voxel_texture_atlas_generator_generate(
    texture_atlas_generator_t *tag, voxel_texture_mesh_t *tmesh,
    voxel_texture2d_t **texs, size_t *texcount);

extern FLYWAVE_VDB_API voxel_texture2d_t *
voxel_texture2d_create(uint32_t width, uint32_t height);
extern FLYWAVE_VDB_API void voxel_texture2d_get_raw_data(voxel_texture2d_t *tex,
                                                         uint8_t **data,
                                                         uint32_t *width,
                                                         uint32_t *height);
extern FLYWAVE_VDB_API void voxel_texture2d_set_raw_data(voxel_texture2d_t *tex,
                                                         uint8_t *data,
                                                         uint32_t width,
                                                         uint32_t height);
extern FLYWAVE_VDB_API voxel_texture2d_t *
voxel_texture2d_duplicate(voxel_texture2d_t *tex);
extern FLYWAVE_VDB_API void voxel_texture2d_fill_pixel(voxel_texture2d_t *tex,
                                                       uint8_t data);
extern FLYWAVE_VDB_API void voxel_texture2d_fill_color(voxel_texture2d_t *tex,
                                                       uint8_t *data);
extern FLYWAVE_VDB_API void
voxel_texture2d_resize(voxel_texture2d_t *tex, uint32_t width, uint32_t height);

extern FLYWAVE_VDB_API voxel_transform_t *
voxel_transform_create_from_voxel_size(double voxelSize);
extern FLYWAVE_VDB_API voxel_transform_t *
voxel_transform_create_from_mat(double *mat);
extern FLYWAVE_VDB_API voxel_transform_t *
voxel_transform_create_from_frustum(double *bbox, double taper, double depth,
                                    double voxelSize);

extern FLYWAVE_VDB_API void voxel_transform_free(voxel_transform_t *vox);
extern FLYWAVE_VDB_API voxel_transform_t *
voxel_transform_duplicate(voxel_transform_t *vox);
extern FLYWAVE_VDB_API _Bool voxel_transform_is_linear(voxel_transform_t *tran);
extern FLYWAVE_VDB_API _Bool
voxel_transform_is_identity(voxel_transform_t *tran);
extern FLYWAVE_VDB_API void
voxel_transform_pre_rotate(voxel_transform_t *tran, double radians, int axis);
extern FLYWAVE_VDB_API void
voxel_transform_pre_translate(voxel_transform_t *tran, double *t);
extern FLYWAVE_VDB_API void
voxel_transform_pre_scale_vector(voxel_transform_t *tran, double *vec);
extern FLYWAVE_VDB_API void voxel_transform_pre_scale(voxel_transform_t *tran,
                                                      double);
extern FLYWAVE_VDB_API void voxel_transform_pre_shear(voxel_transform_t *tran,
                                                      double shear, int axis0,
                                                      int axis1);
extern FLYWAVE_VDB_API void voxel_transform_pre_mult4d(voxel_transform_t *tran,
                                                       double *mat);
extern FLYWAVE_VDB_API void voxel_transform_pre_mult3d(voxel_transform_t *tran,
                                                       double *mat);

extern FLYWAVE_VDB_API void
voxel_transform_post_rotate(voxel_transform_t *tran, double radians, int axis);
extern FLYWAVE_VDB_API void
voxel_transform_post_translate(voxel_transform_t *tran, double *t);
extern FLYWAVE_VDB_API void
voxel_transform_post_scale_vector(voxel_transform_t *tran, double *vec);
extern FLYWAVE_VDB_API void voxel_transform_post_scale(voxel_transform_t *tran,
                                                       double);
extern FLYWAVE_VDB_API void voxel_transform_post_shear(voxel_transform_t *tran,
                                                       double shear, int axis0,
                                                       int axis1);
extern FLYWAVE_VDB_API void voxel_transform_post_mult4d(voxel_transform_t *tran,
                                                        double *mat);
extern FLYWAVE_VDB_API void voxel_transform_post_mult3d(voxel_transform_t *tran,
                                                        double *mat);

extern FLYWAVE_VDB_API void
voxel_transform_voxel_size(voxel_transform_t *tran, double *xyz, double *size);
extern FLYWAVE_VDB_API double
voxel_transform_voxel_volume(voxel_transform_t *tran, double *xyz);

extern FLYWAVE_VDB_API _Bool
voxel_transform_has_uniform_scale(voxel_transform_t *tran);

extern FLYWAVE_VDB_API void
voxel_transform_index_to_world_from_xyz(voxel_transform_t *tran, double *xyz,
                                        double *pos);
extern FLYWAVE_VDB_API void
voxel_transform_index_to_world_from_ijk(voxel_transform_t *tran, int32_t *ijk,
                                        double *pos);
extern FLYWAVE_VDB_API void
voxel_transform_world_to_index_from_xyz(voxel_transform_t *tran, double *xyz,
                                        double *indexs);
extern FLYWAVE_VDB_API void
voxel_transform_world_to_index_cell_centered(voxel_transform_t *tran,
                                             double *xyz, int32_t *coords);
extern FLYWAVE_VDB_API void
voxel_transform_world_to_index_node_centered(voxel_transform_t *tran,
                                             double *xyz, int32_t *coords);

extern FLYWAVE_VDB_API void
voxel_transform_index_to_world_from_coordbox(voxel_transform_t *tran,
                                             int32_t *ibbox, double *bbox);
extern FLYWAVE_VDB_API void
voxel_transform_index_to_world_from_bbox(voxel_transform_t *tran, double *ibbox,
                                         double *bbox);

extern FLYWAVE_VDB_API void
voxel_transform_world_to_index_from_bbox(voxel_transform_t *tran, double *bbox,
                                         double *tbbox);
extern FLYWAVE_VDB_API void
voxel_transform_world_to_index_cell_centered_from_bbox(voxel_transform_t *tran,
                                                       double *bbox,
                                                       int32_t *cbbox);
extern FLYWAVE_VDB_API void
voxel_transform_world_to_index_node_centered_from_bbox(voxel_transform_t *tran,
                                                       double *bbox,
                                                       int32_t *cbbox);

extern FLYWAVE_VDB_API voxel_texture_mesh_t *
voxel_texture_mesh_create_from_triangles(voxel_io_triangle_t *tris, int count);
extern FLYWAVE_VDB_API voxel_texture_mesh_t *voxel_texture_mesh_create();
extern FLYWAVE_VDB_API void voxel_texture_mesh_free(voxel_texture_mesh_t *vox);
extern FLYWAVE_VDB_API voxel_texture_mesh_t *
voxel_texture_mesh_duplicate(voxel_texture_mesh_t *vox);
extern FLYWAVE_VDB_API void
voxel_texture_mesh_get_triangles(voxel_texture_mesh_t *mesh,
                                 voxel_io_triangle_t **tris, int *count,
                                 int node);
extern FLYWAVE_VDB_API void voxel_texture_mesh_lock(voxel_texture_mesh_t *vox,
                                                    _Bool *locked, int count);
extern FLYWAVE_VDB_API void
voxel_texture_mesh_lock_border(voxel_texture_mesh_t *vox);
extern FLYWAVE_VDB_API void
voxel_texture_mesh_unlock_border(voxel_texture_mesh_t *vox);
extern FLYWAVE_VDB_API void
voxel_texture_mesh_quadric_simplify_with_tex(voxel_texture_mesh_t *vox,
                                             uint32_t target);
extern FLYWAVE_VDB_API void
voxel_texture_mesh_quadric_simplify(voxel_texture_mesh_t *vox, uint32_t target);

extern FLYWAVE_VDB_API voxel_mesh_builder_t *voxel_mesh_builder_create();
extern FLYWAVE_VDB_API void voxel_mesh_builder_free(voxel_mesh_builder_t *vox);
extern FLYWAVE_VDB_API void
voxel_mesh_builder_set_name(voxel_mesh_builder_t *vox, const char *name);
extern FLYWAVE_VDB_API void
voxel_mesh_builder_add_mesh_data(voxel_mesh_builder_t *vox,
                                 voxel_pixel_mesh_data_t *data);
extern FLYWAVE_VDB_API void voxel_mesh_builder_add_material_data(
    voxel_mesh_builder_t *vox, voxel_pixel_material_data_t *data, int index);
extern FLYWAVE_VDB_API void
voxel_mesh_builder_add_texture_data(voxel_mesh_builder_t *vox,
                                    voxel_pixel_texture_data_t *data,
                                    const char *name);
extern FLYWAVE_VDB_API _Bool
voxel_mesh_builder_texture_exist(voxel_mesh_builder_t *vox, const char *name);
extern FLYWAVE_VDB_API _Bool
voxel_mesh_builder_material_exist(voxel_mesh_builder_t *vox, int index);
extern FLYWAVE_VDB_API voxel_mesh_t *
voxel_mesh_builder_build_mesh(voxel_mesh_builder_t *vox);

#ifdef __cplusplus
}
#endif
