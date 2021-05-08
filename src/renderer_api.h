#pragma once

#include "voxelizer_api.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _voxel_renderer_t voxel_renderer_t;

extern FLYWAVE_VDB_API voxel_renderer_t *voxel_renderer_create();
extern FLYWAVE_VDB_API void voxel_renderer_free(voxel_renderer_t *vox);

extern FLYWAVE_VDB_API void
voxel_renderer_set_checkpoint_interval(voxel_renderer_t *vox, double inter);
extern FLYWAVE_VDB_API void voxel_renderer_set_timeout(voxel_renderer_t *vox,
                                                       double t);
extern FLYWAVE_VDB_API void
voxel_renderer_set_thread_count(voxel_renderer_t *vox, int threadCount);
extern FLYWAVE_VDB_API void
voxel_renderer_set_input_directory(voxel_renderer_t *vox, const char *path);
extern FLYWAVE_VDB_API void
voxel_renderer_set_output_directory(voxel_renderer_t *vox, const char *path);

extern FLYWAVE_VDB_API void voxel_renderer_setup(voxel_renderer_t *vox);
extern FLYWAVE_VDB_API _Bool voxel_renderer_render_scene(voxel_renderer_t *vox);

extern FLYWAVE_VDB_API _Bool voxel_renderer_frame_buffer(voxel_renderer_t *vox,
                                                         int32_t *resolution,
                                                         uint8_t **buf);
extern FLYWAVE_VDB_API _Bool voxel_renderer_relocate(voxel_renderer_t *vox,
                                                     const char *path,
                                                     bool copyRelocate);
extern FLYWAVE_VDB_API _Bool voxel_renderer_ziparchive(voxel_renderer_t *vox,
                                                       const char *path,
                                                       int compressionLevel);
extern FLYWAVE_VDB_API _Bool voxel_renderer_denoiser(voxel_renderer_t *vox,
                                                     const char *path);

#ifdef __cplusplus
}
#endif
