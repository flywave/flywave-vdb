#include "voxelizer_api.h"
#include "voxelizer_api_impl.hh"

#include "micronizer.hh"
#include "voxel_mesh.hh"

using namespace flywave;

#ifdef __cplusplus
extern "C" {
#endif

FLYWAVE_VDB_API voxelizer_t *voxelizer_create() { return nullptr; }

FLYWAVE_VDB_API void voxelizer_free(voxelizer_t *vox) {}

FLYWAVE_VDB_API voxelizer_t *voxelizer_duplicate(voxelizer_t *vox) {
  return nullptr;
}

#ifdef __cplusplus
}
#endif
