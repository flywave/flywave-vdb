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

typedef struct _voxelizer_t voxelizer_t;

extern FLYWAVE_VDB_API voxelizer_t *voxelizer_create();
extern FLYWAVE_VDB_API void voxelizer_free(voxelizer_t *vox);
extern FLYWAVE_VDB_API voxelizer_t *voxelizer_duplicate(voxelizer_t *vox);

#ifdef __cplusplus
}
#endif
