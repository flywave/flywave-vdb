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

typedef struct _voxel_pixel_t voxel_pixel_t;
typedef struct _io_vertex_t io_vertex_t;
typedef struct _io_triangle_t io_triangle_t;
typedef struct _textute_foundry_t textute_foundry_t;

extern FLYWAVE_VDB_API voxel_pixel_t *voxel_pixel_create();
extern FLYWAVE_VDB_API void voxel_pixel_free(voxel_pixel_t *vox);
extern FLYWAVE_VDB_API voxel_pixel_t *voxel_pixel_duplicate(voxel_pixel_t *vox);

extern FLYWAVE_VDB_API void make_triangles(voxel_pixel_t *vox);

#ifdef __cplusplus
}
#endif
