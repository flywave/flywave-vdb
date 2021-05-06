#include "voxelizer_api.h"
#include "voxelizer_api_impl.hh"

#include "voxel_mesh.hh"
#include "voxel_pixel_sampler.hh"

using namespace flywave;

#ifdef __cplusplus
extern "C" {
#endif

FLYWAVE_VDB_API voxel_pixel_t *voxel_pixel_create() { return nullptr; }

FLYWAVE_VDB_API void voxel_pixel_free(voxel_pixel_t *vox) {}

FLYWAVE_VDB_API voxel_pixel_t *voxel_pixel_duplicate(voxel_pixel_t *vox) {
  return nullptr;
}

FLYWAVE_VDB_API _Bool voxel_pixel_read(voxel_pixel_t *vox, const char *path) {
  return false;
}

FLYWAVE_VDB_API _Bool voxel_pixel_write(voxel_pixel_t *vox, const char *path) {
  return false;
}

FLYWAVE_VDB_API void voxel_pixel_composite(voxel_pixel_t *vox,
                                           voxel_pixel_t *tg, uint32_t type) {}

FLYWAVE_VDB_API void voxel_pixel_clear(voxel_pixel_t *vox) {}

FLYWAVE_VDB_API _Bool voxel_pixel_ray_test(voxel_pixel_t *vox,
                                           double *ray_origin,
                                           double *ray_direction, double *p) {
  return false;
}

FLYWAVE_VDB_API void voxel_pixel_voxel_resolution(voxel_pixel_t *vox,
                                                  double *mat) {}

FLYWAVE_VDB_API vdb_float_grid_t *
voxel_pixel_get_voxel_grid(voxel_pixel_t *vox) {
    return nullptr;
}

FLYWAVE_VDB_API vdb_pixel_grid_t *
voxel_pixel_get_pixel_grid(voxel_pixel_t *vox) {
  return nullptr;
}

FLYWAVE_VDB_API void voxel_pixel_set_voxel_grid(voxel_pixel_t *vox,
                                                vdb_float_grid_t *vg) {}

FLYWAVE_VDB_API void voxel_pixel_set_pixel_grid(voxel_pixel_t *vox,
                                                vdb_float_grid_t *vg) {}

FLYWAVE_VDB_API _Bool voxel_pixel_is_empty(voxel_pixel_t *vox) { return false; }

FLYWAVE_VDB_API voxel_pixel_materials_t *
voxel_pixel_get_materials(voxel_pixel_t *vox) {}

FLYWAVE_VDB_API void voxel_pixel_set_materials(voxel_pixel_t *vox,
                                               voxel_pixel_materials_t *mtls) {}

FLYWAVE_VDB_API void make_triangles(voxel_pixel_t *vox) {}

#ifdef __cplusplus
}
#endif
