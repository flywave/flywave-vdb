#include "voxelizer_api.h"
#include "voxelizer_api_impl.hh"

#include "micronizer.hh"
#include "voxel_mesh.hh"

using namespace flywave;

#ifdef __cplusplus
extern "C" {
#endif

FLYWAVE_VDB_API voxel_pixel_t *voxel_pixel_create() { return nullptr; }

FLYWAVE_VDB_API void voxel_pixel_free(voxel_pixel_t *vox) {}

FLYWAVE_VDB_API voxel_pixel_t *voxel_pixel_duplicate(voxel_pixel_t *vox) {
  return nullptr;
}

#ifdef __cplusplus
}
#endif
