#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if defined(WIN32) || defined(WINDOWS) || defined(_WIN32) || defined(_WINDOWS)
#define FLYWAVE_VDB_API __declspec(dllexport)
#else
#define FLYWAVE_VDB_API
#endif

typedef struct _vdb_grid_t vdb_grid_t;

extern FLYWAVE_VDB_API vdb_grid_t *vdb_create();
extern FLYWAVE_VDB_API void vdb_free(vdb_grid_t *grid);
extern FLYWAVE_VDB_API vdb_grid_t *vdb_duplicate(vdb_grid_t *grid);

extern FLYWAVE_VDB_API bool vdb_read(vdb_grid_t *grid, const char *filename);
extern FLYWAVE_VDB_API bool vdb_write(vdb_grid_t *grid, const char *filename);

extern FLYWAVE_VDB_API bool vdb_from_points(vdb_grid_t *grid, double *vPoints,
                                            int pCount, double *vRadius,
                                            int rCount, double voxelSize,
                                            double bandwidth);
extern FLYWAVE_VDB_API bool vdb_from_mesh(vdb_grid_t *grid, float *vPoints,
                                          int vCount, int *vFaces, int fCount,
                                          double voxelSize, double bandwidth);

extern FLYWAVE_VDB_API void vdb_to_mesh(vdb_grid_t *grid);
extern FLYWAVE_VDB_API void
vdb_to_mesh_settings(vdb_grid_t *grid, double isovalue, double adaptivity);

extern FLYWAVE_VDB_API float *vdb_vertex_buffer(vdb_grid_t *grid, int *size);
extern FLYWAVE_VDB_API int *vdb_face_buffer(vdb_grid_t *grid, int *size);

extern FLYWAVE_VDB_API bool vdb_transform(vdb_grid_t *grid, double *matrix,
                                          int mCount);

extern FLYWAVE_VDB_API void vdb_union(vdb_grid_t *grid, vdb_grid_t *csgGrid);
extern FLYWAVE_VDB_API void vdb_difference(vdb_grid_t *grid,
                                           vdb_grid_t *csgGrid);
extern FLYWAVE_VDB_API void vdb_intersection(vdb_grid_t *grid,
                                             vdb_grid_t *csgGrid);

extern FLYWAVE_VDB_API void vdb_offset(vdb_grid_t *grid, double amount);
extern FLYWAVE_VDB_API void vdb_offset_mask(vdb_grid_t *grid, double amount,
                                            vdb_grid_t *mask, double min,
                                            double max, bool invert);
extern FLYWAVE_VDB_API void vdb_smooth(vdb_grid_t *grid, int type,
                                       int iterations, int width);
extern FLYWAVE_VDB_API void vdb_smooth_mask(vdb_grid_t *grid, int type,
                                            int iterations, int width,
                                            vdb_grid_t *mask, double min,
                                            double max, bool invert);
extern FLYWAVE_VDB_API void vdb_blend(vdb_grid_t *bGrid, vdb_grid_t *eGrid,
                                      double bPosition, double bEnd);
extern FLYWAVE_VDB_API void vdb_blend_mask(vdb_grid_t *bGrid, vdb_grid_t *eGrid,
                                           double bPosition, double bEnd,
                                           vdb_grid_t *mask, double min,
                                           double max, bool invert);

extern FLYWAVE_VDB_API float *
vdb_closest_point(vdb_grid_t *grid, float *vPoints, int vCount, int *rSize);

#ifdef __cplusplus
}
#endif
