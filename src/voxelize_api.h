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

typedef struct _voxelize_t voxelize_t;

extern FLYWAVE_VDB_API voxelize_t *voxelize_create();
extern FLYWAVE_VDB_API void voxelize_free(voxelize_t *vox);
extern FLYWAVE_VDB_API voxelize_t *voxelize_duplicate(voxelize_t *vox);

#ifdef __cplusplus
}
#endif
