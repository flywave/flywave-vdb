#include "voxelizer_api.h"
#include "texture_atlas.hh"
#include "texture_mesh.hh"
#include "voxelizer_api_impl.hh"

#include "grid_api_impl.hh"
#include "repacker.hh"
#include "voxel_mesh.hh"
#include "voxel_pixel_sampler.hh"

using namespace flywave;

#ifdef __cplusplus
extern "C" {
#endif

FLYWAVE_VDB_API voxel_pixel_t *voxel_pixel_create() {
  return new voxel_pixel_t{voxel_pixel::create_voxel_pixel()};
}

FLYWAVE_VDB_API void voxel_pixel_free(voxel_pixel_t *vox) { delete vox; }

FLYWAVE_VDB_API voxel_pixel_t *voxel_pixel_duplicate(voxel_pixel_t *vox) {
  return new voxel_pixel_t{std::make_shared<voxel_pixel>(vox->ptr.get())};
}

FLYWAVE_VDB_API _Bool voxel_pixel_read(voxel_pixel_t *vox, const char *path) {
  return vox->ptr->read(path);
}

FLYWAVE_VDB_API _Bool voxel_pixel_write(voxel_pixel_t *vox, const char *path) {
  return vox->ptr->write(path);
}

FLYWAVE_VDB_API void voxel_pixel_composite(voxel_pixel_t *vox,
                                           voxel_pixel_t *tg, uint32_t type) {
  vox->ptr->composite(*tg->ptr,
                      static_cast<flywave::voxel_pixel::composite_type>(type));
}

FLYWAVE_VDB_API void voxel_pixel_clear(voxel_pixel_t *vox) {
  vox->ptr->clear();
}

FLYWAVE_VDB_API _Bool voxel_pixel_ray_test(voxel_pixel_t *vox,
                                           double *ray_origin,
                                           double *ray_direction, double *p) {
  vdb::math::Ray<double> ray(
      vdb::Vec3d(ray_origin[0], ray_origin[1], ray_origin[2]),
      vdb::Vec3d(ray_direction[0], ray_direction[1], ray_direction[2]));
  vdb::Vec3d query;
  auto f = vox->ptr->ray_test(ray, query);
  p[0] = query.x();
  p[1] = query.y();
  p[2] = query.z();
  return f;
}

FLYWAVE_VDB_API voxel_transform_t *
voxel_pixel_voxel_resolution(voxel_pixel_t *vox) {
  return new voxel_transform_t{vox->ptr->voxel_resolution()};
}

FLYWAVE_VDB_API vdb_float_grid_t *
voxel_pixel_get_voxel_grid(voxel_pixel_t *vox) {
  return new vdb_float_grid_t{
      std::make_shared<vdb_float_grid>(vox->ptr->get_voxel_grid())};
}

FLYWAVE_VDB_API vdb_pixel_grid_t *
voxel_pixel_get_pixel_grid(voxel_pixel_t *vox) {
  return new vdb_pixel_grid_t{
      std::make_shared<vdb_pixel_grid>(vox->ptr->get_pixel_grid())};
}

FLYWAVE_VDB_API void voxel_pixel_set_voxel_grid(voxel_pixel_t *vox,
                                                vdb_float_grid_t *vg) {
  vox->ptr->set_voxel_grid(vg->ptr->grid());
}

FLYWAVE_VDB_API void voxel_pixel_set_pixel_grid(voxel_pixel_t *vox,
                                                vdb_pixel_grid_t *vg) {
  vox->ptr->set_pixel_grid(vg->ptr->grid());
}

FLYWAVE_VDB_API _Bool voxel_pixel_is_empty(voxel_pixel_t *vox) {
  return vox->ptr->is_empty();
}

FLYWAVE_VDB_API voxel_pixel_materials_t *
voxel_pixel_get_materials(voxel_pixel_t *vox) {
  return new voxel_pixel_materials_t{vox->ptr->materials()};
}

FLYWAVE_VDB_API void voxel_pixel_set_materials(voxel_pixel_t *vox,
                                               voxel_pixel_materials_t *mtls) {
  vox->ptr->set_materials(mtls->mtls);
}

FLYWAVE_VDB_API vdb_pixel_grid_t *
voxel_pixel_extract_color(voxel_pixel_t *vox, voxel_pixel_t *svox) {
  return new vdb_pixel_grid_t{
      std::make_shared<vdb_pixel_grid>(vox->ptr->extract_color(*(svox->ptr)))};
}

FLYWAVE_VDB_API void voxel_pixel_fill_color(voxel_pixel_t *vox,
                                            voxel_pixel_t *svox,
                                            vdb_pixel_grid_t *colors) {
  return vox->ptr->fill_color(*(svox->ptr), colors->ptr->grid());
}

FLYWAVE_VDB_API void voxel_pixel_eval_max_min_elevation(voxel_pixel_t *vox,
                                                        double *bboxin,
                                                        double *bboxout) {
  vdb::BBoxd _in(vdb::Vec3d{bboxin[0], bboxin[1], bboxin[2]},
                 vdb::Vec3d{bboxin[3], bboxin[4], bboxin[5]});
  auto _out = vox->ptr->eval_max_min_elevation(_in);
  bboxout[0] = _out.min().x();
  bboxout[1] = _out.min().y();
  bboxout[2] = _out.min().z();

  bboxout[3] = _out.max().x();
  bboxout[4] = _out.max().y();
  bboxout[5] = _out.max().z();
}

static_assert(sizeof(voxel_io_vertex_t) == sizeof(voxel_io_vertex), "");
static_assert(sizeof(voxel_io_triangle_t) == sizeof(voxel_io_triangle), "");

FLYWAVE_VDB_API void
voxel_pixel_make_triangles(voxel_pixel_t *vox, voxel_io_triangle_t **tris,
                           size_t *tricount, double *mat, size_t tex_offset,
                           size_t mtl_offset, voxel_border_lock_t *bl,
                           voxel_filter_triangle_t *ftri, double fquality,
                           double isovalue, double adapter) {
  std::vector<struct voxel_io_triangle> vtriangles;
  make_triangles(vtriangles, *vox->ptr, vdb::Mat4d(mat), tex_offset, mtl_offset,
                 bl->ptr, ftri->ptr, fquality, isovalue, adapter);

  *tricount = vtriangles.size();
  *tris = (voxel_io_triangle_t *)malloc(sizeof(voxel_io_triangle_t) *
                                        vtriangles.size());
  memcpy(tris, vtriangles.data(),
         sizeof(voxel_io_triangle_t) * vtriangles.size());
}

FLYWAVE_VDB_API void voxel_pixel_make_triangles_simple(
    voxel_pixel_t *vox, voxel_io_triangle_t **tris, size_t *tricount,
    double *mat, size_t tex_offset, size_t mtl_offset, double fquality,
    double isovalue, double adapter) {
  std::vector<struct voxel_io_triangle> vtriangles;
  make_triangles(vtriangles, *vox->ptr, vdb::Mat4d(mat), tex_offset, mtl_offset,
                 fquality, isovalue, adapter);

  *tricount = vtriangles.size();
  *tris = (voxel_io_triangle_t *)malloc(sizeof(voxel_io_triangle_t) *
                                        vtriangles.size());
  memcpy(tris, vtriangles.data(),
         sizeof(voxel_io_triangle_t) * vtriangles.size());
}

FLYWAVE_VDB_API texture_atlas_generator_t *
voxel_pixel_make_texture_atlas_generator(voxel_pixel_t *vox,
                                         voxel_texture_mesh_t *tmesh,
                                         double *mat, float pixel_pad) {
  auto repacker = std::make_shared<textute_repacker>(
      vox->ptr->get_voxel_grid(), vox->ptr->get_pixel_grid(), vdb::Mat4d(mat),
      pixel_pad);

  auto tgen = std::make_shared<texture_atlas_generator>();

  tgen->push_atlas(repacker);
  tgen->generate(*tmesh->mesh, *tmesh->mesh);

  return new texture_atlas_generator_t{tgen};
}

#ifdef __cplusplus
}
#endif
