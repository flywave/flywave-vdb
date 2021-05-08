#include "renderer_api.h"
#include "pbr_renderer.hh"

using namespace flywave;

#ifdef __cplusplus
extern "C" {
#endif

struct _voxel_renderer_t {
  std::shared_ptr<flywave::pbr_renderer> ptr;
};

FLYWAVE_VDB_API voxel_renderer_t *voxel_renderer_create() {
  return new voxel_renderer_t{std::make_shared<flywave::pbr_renderer>()};
}

FLYWAVE_VDB_API void voxel_renderer_free(voxel_renderer_t *vox) { delete vox; }

FLYWAVE_VDB_API void
voxel_renderer_set_checkpoint_interval(voxel_renderer_t *vox, double inter) {
  vox->ptr->set_checkpoint_interval(inter);
}

FLYWAVE_VDB_API void voxel_renderer_set_timeout(voxel_renderer_t *vox,
                                                double t) {
  vox->ptr->set_timeout(t);
}

FLYWAVE_VDB_API void voxel_renderer_set_thread_count(voxel_renderer_t *vox,
                                                     int threadCount) {
  vox->ptr->set_thread_count(threadCount);
}

FLYWAVE_VDB_API void voxel_renderer_set_input_directory(voxel_renderer_t *vox,
                                                        const char *path) {
  vox->ptr->set_input_directory(path);
}

FLYWAVE_VDB_API void voxel_renderer_set_output_directory(voxel_renderer_t *vox,
                                                         const char *path) {
  vox->ptr->set_output_directory(path);
}

FLYWAVE_VDB_API void voxel_renderer_setup(voxel_renderer_t *vox) {
  vox->ptr->setup();
}

FLYWAVE_VDB_API _Bool voxel_renderer_render_scene(voxel_renderer_t *vox) {
  return vox->ptr->render_scene();
}

FLYWAVE_VDB_API _Bool voxel_renderer_frame_buffer(voxel_renderer_t *vox,
                                                  int32_t *resolution,
                                                  uint8_t **buf) {
  Tungsten::Vec2i res(resolution);
  auto fbuf = vox->ptr->frame_buffer(res);
  *buf = (uint8_t *)malloc(res.x() * res.y() * 3);
  memcpy(*buf, fbuf.get(), res.x() * res.y() * 3);
  return true;
}

FLYWAVE_VDB_API _Bool voxel_renderer_relocate(voxel_renderer_t *vox,
                                              const char *path,
                                              bool copyRelocate) {
  return vox->ptr->relocate(path, copyRelocate);
}

FLYWAVE_VDB_API _Bool voxel_renderer_ziparchive(voxel_renderer_t *vox,
                                                const char *path,
                                                int compressionLevel) {
  return vox->ptr->ziparchive(path, compressionLevel);
}

FLYWAVE_VDB_API _Bool voxel_renderer_denoiser(voxel_renderer_t *vox,
                                              const char *path) {
  return vox->ptr->denoiser(path);
}

#ifdef __cplusplus
}
#endif
