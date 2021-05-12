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
typedef struct _voxel_texture_atlas_generator_t voxel_texture_atlas_generator_t;
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
typedef struct _voxel_clip_box_createor_t voxel_clip_box_createor_t;

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
extern FLYWAVE_VDB_API _Bool voxel_pixel_ray_tests(voxel_pixel_t *vox,
                                                   double *ray_origin,
                                                   double *ray_direction,
                                                   double *p, size_t count);
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
extern FLYWAVE_VDB_API int64_t voxel_pixel_get_memory_size(voxel_pixel_t *vox);

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

extern FLYWAVE_VDB_API void
voxel_pixel_materials_free(voxel_pixel_materials_t *mtls);

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

extern FLYWAVE_VDB_API voxel_texture_atlas_generator_t *
voxel_pixel_make_texture_atlas_generator(voxel_pixel_t *vox, double *mat,
                                         float pixel_pad);

extern FLYWAVE_VDB_API void
voxel_texture_atlas_generator_free(voxel_texture_atlas_generator_t *tag);

extern FLYWAVE_VDB_API void voxel_texture_atlas_generator_generate(
    voxel_texture_atlas_generator_t *tag, voxel_texture_mesh_t *tmesh,
    voxel_texture2d_t ***texs, size_t *texcount);

extern FLYWAVE_VDB_API voxel_texture2d_t *
voxel_texture2d_create(uint32_t width, uint32_t height);
extern FLYWAVE_VDB_API void voxel_texture2d_free(voxel_texture2d_t *tex);
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
extern FLYWAVE_VDB_API voxel_texture_mesh_t *
voxel_texture_mesh_create_from_mesh_datas(voxel_pixel_mesh_data_t **mdatas,
                                          int count);
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
extern FLYWAVE_VDB_API void voxel_texture_mesh_save(voxel_texture_mesh_t *vox,
                                                    const char *path,
                                                    uint32_t node);

extern FLYWAVE_VDB_API voxel_mesh_builder_t *voxel_mesh_builder_create();
extern FLYWAVE_VDB_API void voxel_mesh_builder_free(voxel_mesh_builder_t *vox);
extern FLYWAVE_VDB_API void
voxel_mesh_builder_set_name(voxel_mesh_builder_t *vox, const char *name);
extern FLYWAVE_VDB_API const char *
voxel_mesh_builder_get_name(voxel_mesh_builder_t *vox);
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
extern FLYWAVE_VDB_API voxel_texture_mesh_t *
voxel_mesh_builder_build_texture_mesh(voxel_mesh_builder_t *vox);

extern FLYWAVE_VDB_API void voxel_mesh_free(voxel_mesh_t *m);
extern FLYWAVE_VDB_API voxel_pixel_t *
voxel_mesh_to_voxel_pixel(voxel_mesh_t *m, voxel_pixel_materials_t *mtls,
                          uint16_t local_feature, float precision,
                          voxel_clip_box_createor_t *creator, int32_t type,
                          double *matrix);

typedef struct _c_material_data_t {
  uint8_t _material_id;

  uint32_t _type;
  uint16_t mode;

  uint8_t color_r;
  uint8_t color_g;
  uint8_t color_b;

  uint8_t ambient_r;
  uint8_t ambient_g;
  uint8_t ambient_b;

  uint8_t emissive_r;
  uint8_t emissive_g;
  uint8_t emissive_b;

  uint8_t specular_r;
  uint8_t specular_g;
  uint8_t specular_b;

  float opacity;

  float shininess;

  float metallic;
  float roughness;
  float reflectance;

  float clearcoat_thickness;
  float clearcoat_roughness;

  float anisotropy;
  float anisotropy_rotation;
} c_material_data_t;

extern FLYWAVE_VDB_API voxel_pixel_material_data_t *
voxel_pixel_material_data_create(c_material_data_t data);
extern FLYWAVE_VDB_API void
voxel_pixel_material_data_free(voxel_pixel_material_data_t *vox);
extern FLYWAVE_VDB_API c_material_data_t
voxel_pixel_material_data_get(voxel_pixel_material_data_t *vox);
extern FLYWAVE_VDB_API void
voxel_pixel_material_data_set(voxel_pixel_material_data_t *vox,
                              c_material_data_t data);

typedef struct _c_texture_data_t {
  size_t width;
  size_t height;
  uint16_t format;
  uint8_t *data;
} c_texture_data_t;

extern FLYWAVE_VDB_API voxel_pixel_texture_data_t *
voxel_pixel_texture_data_create(c_texture_data_t t);
extern FLYWAVE_VDB_API void
voxel_pixel_texture_data_free(voxel_pixel_texture_data_t *vox);
extern FLYWAVE_VDB_API c_texture_data_t
voxel_pixel_texture_data_get(voxel_pixel_texture_data_t *vox);
extern FLYWAVE_VDB_API void
voxel_pixel_texture_data_set(voxel_pixel_texture_data_t *vox,
                             c_texture_data_t data);

struct _c_mesh_data_mtl_t {
  int mtl;
  size_t f_count;
  size_t n_count;
  size_t t_count;

  uint32_t *faces;
  uint32_t *normals;
  uint32_t *texcoords;
};

typedef struct _c_mesh_data_t {
  float *vertices;
  size_t v_count;

  float *texcoords;
  size_t t_count;

  float *normals;
  size_t n_count;

  struct _c_mesh_data_mtl_t *mtl_map;
  size_t mtl_count;

} c_mesh_data_t;

extern FLYWAVE_VDB_API voxel_pixel_mesh_data_t *
voxel_pixel_mesh_data_create(c_mesh_data_t data);
extern FLYWAVE_VDB_API void
voxel_pixel_mesh_data_free(voxel_pixel_mesh_data_t *vox);

extern FLYWAVE_VDB_API c_mesh_data_t
voxel_pixel_mesh_data_get(voxel_pixel_mesh_data_t *vox);
extern FLYWAVE_VDB_API void
voxel_pixel_mesh_data_set(voxel_pixel_mesh_data_t *vox, c_mesh_data_t data);

extern FLYWAVE_VDB_API void voxel_pixel_c_mesh_data_alloc(c_mesh_data_t *cm,
                                                          size_t mtlcount);
extern FLYWAVE_VDB_API void voxel_pixel_c_mesh_data_free(c_mesh_data_t *cm);

extern FLYWAVE_VDB_API voxel_border_lock_t *voxel_border_lock_create(void *ctx);
extern FLYWAVE_VDB_API void voxel_border_lock_free(voxel_border_lock_t *vox);

extern FLYWAVE_VDB_API voxel_filter_triangle_t *
voxel_filter_triangle_create(void *ctx);
extern FLYWAVE_VDB_API void
voxel_filter_triangle_free(voxel_filter_triangle_t *vox);

extern FLYWAVE_VDB_API voxel_clip_box_createor_t *
voxel_clip_box_createor_create(void *ctx);
extern FLYWAVE_VDB_API void
voxel_clip_box_createor_free(voxel_clip_box_createor_t *vox);

#ifdef __cplusplus
}
#endif
