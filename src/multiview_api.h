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

/* ─── Transfer Function ─────────────────────────────────────── */

typedef struct _vdb_transfer_function_t vdb_transfer_function_t;

FLYWAVE_VDB_API vdb_transfer_function_t *vdb_transfer_function_create(void);
FLYWAVE_VDB_API void vdb_transfer_function_free(vdb_transfer_function_t *tf);
FLYWAVE_VDB_API void vdb_transfer_function_add_point(
    vdb_transfer_function_t *tf,
    float scalar, float r, float g, float b, float a);
FLYWAVE_VDB_API void vdb_transfer_function_map(
    vdb_transfer_function_t *tf,
    float scalar, float *r, float *g, float *b, float *a);
FLYWAVE_VDB_API void vdb_transfer_function_build(vdb_transfer_function_t *tf);

/* ─── Volume Ray March ──────────────────────────────────────── */

typedef struct _vdb_float_grid_t vdb_float_grid_t;

FLYWAVE_VDB_API bool vdb_grid_ray_march(
    vdb_float_grid_t *grid,
    double ox, double oy, double oz,
    double dx, double dy, double dz,
    vdb_transfer_function_t *tfn,
    double step_size, double max_dist,
    float *out_r, float *out_g, float *out_b, float *out_a,
    double *out_depth);

FLYWAVE_VDB_API bool vdb_grid_ray_march_many(
    vdb_float_grid_t *grid,
    const double *origins,  /* [n][3] */
    const double *dirs,     /* [n][3] */
    vdb_transfer_function_t *tfn,
    double step_size, double max_dist,
    int n,
    float *colors,       /* [n][4]  r,g,b,a */
    double *depths);     /* [n] */

/* ─── Multi-View Camera ─────────────────────────────────────── */

typedef struct {
    double eye[3];
    double target[3];
    double up[3];
    double fov;
} vdb_camera_t;

FLYWAVE_VDB_API void vdb_camera_look_at(
    vdb_camera_t *cam,
    double eye_x, double eye_y, double eye_z,
    double target_x, double target_y, double target_z,
    double up_x, double up_y, double up_z,
    double fov);

FLYWAVE_VDB_API void vdb_camera_orbit(
    vdb_camera_t *cam,
    double center_x, double center_y, double center_z,
    double radius, double azimuth_deg, double elevation_deg,
    double fov);

FLYWAVE_VDB_API int vdb_camera_orbit_path(
    vdb_camera_t *cameras, int num_cameras,
    double center_x, double center_y, double center_z,
    double radius, double fov);

#ifdef __cplusplus
}
#endif
