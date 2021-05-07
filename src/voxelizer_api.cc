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
voxel_pixel_make_texture_atlas_generator(voxel_pixel_t *vox, double *mat,
                                         float pixel_pad) {
  auto repacker = std::make_shared<textute_repacker>(
      vox->ptr->get_voxel_grid(), vox->ptr->get_pixel_grid(), vdb::Mat4d(mat),
      pixel_pad);

  auto tgen = std::make_shared<texture_atlas_generator>();

  tgen->push_atlas(repacker);

  return new texture_atlas_generator_t{tgen};
}

FLYWAVE_VDB_API void
voxel_texture_atlas_generator_free(texture_atlas_generator_t *tag) {
  delete tag;
}

FLYWAVE_VDB_API void voxel_texture_atlas_generator_generate(
    texture_atlas_generator_t *tag, voxel_texture_mesh_t *tmesh,
    voxel_texture2d_t **texs, size_t *texcount) {
  auto texvecs = tag->ptr->generate(*tmesh->mesh, *tmesh->mesh);

  *texcount = texvecs.size();
  *texs =
      (voxel_texture2d_t *)malloc(sizeof(voxel_texture2d_t) * texvecs.size());
  for (int i = 0; i < texvecs.size(); i++) {
    texs[i]->ptr = texvecs[i];
  }
}

FLYWAVE_VDB_API voxel_texture2d_t *voxel_texture2d_create(uint32_t width,
                                                          uint32_t height) {
  return new voxel_texture2d_t{
      flywave::texture2d<flywave::vdb::math::Vec4<uint8_t>>::create(
          std::make_pair(width, height))};
}

FLYWAVE_VDB_API void voxel_texture2d_get_raw_data(voxel_texture2d_t *tex,
                                                  uint8_t **data,
                                                  uint32_t *width,
                                                  uint32_t *height) {
  *width = tex->ptr->width();
  *height = tex->ptr->height();
  *data = (uint8_t *)malloc(tex->ptr->bytes());
  memcpy(*data, tex->ptr->raw_data(), tex->ptr->bytes());
}

FLYWAVE_VDB_API void voxel_texture2d_set_raw_data(voxel_texture2d_t *tex,
                                                  uint8_t *data, uint32_t width,
                                                  uint32_t height) {
  tex->ptr->resize(std::make_pair(width, height));
  memcpy(tex->ptr->raw_data(), data, tex->ptr->bytes());
}

FLYWAVE_VDB_API voxel_texture2d_t *
voxel_texture2d_duplicate(voxel_texture2d_t *tex) {
  return new voxel_texture2d_t{tex->ptr->duplicate()};
}

FLYWAVE_VDB_API void voxel_texture2d_fill_pixel(voxel_texture2d_t *tex,
                                                uint8_t data) {
  tex->ptr->fill_pixel(data);
}

FLYWAVE_VDB_API void voxel_texture2d_fill_color(voxel_texture2d_t *tex,
                                                uint8_t *data) {
  tex->ptr->fill_color(flywave::vdb::math::Vec4<uint8_t>(data));
}

FLYWAVE_VDB_API void voxel_texture2d_resize(voxel_texture2d_t *tex,
                                            uint32_t width, uint32_t height) {
  tex->ptr->resize(std::make_pair(width, height));
}

FLYWAVE_VDB_API voxel_transform_t *
voxel_transform_create_from_voxel_size(double voxelSize) {
  return new voxel_transform_t{
      flywave::vdb::math::Transform::createLinearTransform(voxelSize)};
}

FLYWAVE_VDB_API voxel_transform_t *
voxel_transform_create_from_mat(double *mat) {
  return new voxel_transform_t{
      flywave::vdb::math::Transform::createLinearTransform(vdb::Mat4d(mat))};
}

FLYWAVE_VDB_API voxel_transform_t *
voxel_transform_create_from_frustum(double *bbox, double taper, double depth,
                                    double voxelSize) {
  vdb::BBoxd b(vdb::Vec3d{bbox[0], bbox[1], bbox[2]},
               vdb::Vec3d{bbox[3], bbox[4], bbox[5]});
  return new voxel_transform_t{
      flywave::vdb::math::Transform::createFrustumTransform(b, taper, depth,
                                                            voxelSize)};
}

FLYWAVE_VDB_API void voxel_transform_free(voxel_transform_t *vox) {
  delete vox;
}

FLYWAVE_VDB_API voxel_transform_t *
voxel_transform_duplicate(voxel_transform_t *vox) {
  return new voxel_transform_t{vox->ptr->copy()};
}

FLYWAVE_VDB_API _Bool voxel_transform_is_linear(voxel_transform_t *tran) {
  return tran->ptr->isLinear();
}

FLYWAVE_VDB_API _Bool voxel_transform_is_identity(voxel_transform_t *tran) {
  return tran->ptr->isIdentity();
}

FLYWAVE_VDB_API void voxel_transform_pre_rotate(voxel_transform_t *tran,
                                                double radians, int axis) {
  tran->ptr->preRotate(radians, static_cast<vdb::math::Axis>(axis));
}

FLYWAVE_VDB_API void voxel_transform_pre_translate(voxel_transform_t *tran,
                                                   double *vec) {
  tran->ptr->preTranslate(vdb::Vec3d(vec));
}

FLYWAVE_VDB_API void voxel_transform_pre_scale_vector(voxel_transform_t *tran,
                                                      double *vec) {
  tran->ptr->preScale(vdb::Vec3d(vec));
}

FLYWAVE_VDB_API void voxel_transform_pre_scale(voxel_transform_t *tran,
                                               double s) {
  tran->ptr->preScale(s);
}

FLYWAVE_VDB_API void voxel_transform_pre_shear(voxel_transform_t *tran,
                                               double shear, int axis0,
                                               int axis1) {
  tran->ptr->preShear(shear, static_cast<vdb::math::Axis>(axis0),
                      static_cast<vdb::math::Axis>(axis1));
}

FLYWAVE_VDB_API void voxel_transform_pre_mult4d(voxel_transform_t *tran,
                                                double *mat) {
  tran->ptr->preMult(vdb::Mat4d(mat));
}

FLYWAVE_VDB_API void voxel_transform_pre_mult3d(voxel_transform_t *tran,
                                                double *mat) {
  tran->ptr->preMult(vdb::Mat3d(mat));
}

FLYWAVE_VDB_API void voxel_transform_post_rotate(voxel_transform_t *tran,
                                                 double radians, int axis) {
  tran->ptr->postRotate(radians, static_cast<vdb::math::Axis>(axis));
}

FLYWAVE_VDB_API void voxel_transform_post_translate(voxel_transform_t *tran,
                                                    double *vec) {
  tran->ptr->postTranslate(vdb::Vec3d(vec));
}
FLYWAVE_VDB_API void voxel_transform_post_scale_vector(voxel_transform_t *tran,
                                                       double *vec) {
  tran->ptr->postScale(vdb::Vec3d(vec));
}

FLYWAVE_VDB_API void voxel_transform_post_scale(voxel_transform_t *tran,
                                                double s) {
  tran->ptr->postScale(s);
}

FLYWAVE_VDB_API void voxel_transform_post_shear(voxel_transform_t *tran,
                                                double shear, int axis0,
                                                int axis1) {
  tran->ptr->postShear(shear, static_cast<vdb::math::Axis>(axis0),
                       static_cast<vdb::math::Axis>(axis1));
}

FLYWAVE_VDB_API void voxel_transform_post_mult4d(voxel_transform_t *tran,
                                                 double *mat) {
  tran->ptr->postMult(vdb::Mat4d(mat));
}

FLYWAVE_VDB_API void voxel_transform_post_mult3d(voxel_transform_t *tran,
                                                 double *mat) {
  tran->ptr->postMult(vdb::Mat3d(mat));
}

FLYWAVE_VDB_API void voxel_transform_voxel_size(voxel_transform_t *tran,
                                                double *xyz, double *size) {
  if (xyz == nullptr) {
    auto s = tran->ptr->voxelSize();
    size[0] = s.x();
    size[1] = s.y();
    size[2] = s.z();
  } else {
    auto s = tran->ptr->voxelSize(vdb::Vec3d(xyz));
    size[0] = s.x();
    size[1] = s.y();
    size[2] = s.z();
  }
}

FLYWAVE_VDB_API double voxel_transform_voxel_volume(voxel_transform_t *tran,
                                                    double *xyz) {
  if (xyz == nullptr) {
    return tran->ptr->voxelVolume();
  } else {
    return tran->ptr->voxelVolume(vdb::Vec3d(xyz));
  }
}

FLYWAVE_VDB_API _Bool
voxel_transform_has_uniform_scale(voxel_transform_t *tran) {
  return tran->ptr->hasUniformScale();
}

FLYWAVE_VDB_API void
voxel_transform_index_to_world_from_xyz(voxel_transform_t *tran, double *xyz,
                                        double *pos) {
  auto s = tran->ptr->indexToWorld(vdb::Vec3d(xyz));
  pos[0] = s.x();
  pos[1] = s.y();
  pos[2] = s.z();
}

FLYWAVE_VDB_API void
voxel_transform_index_to_world_from_ijk(voxel_transform_t *tran, int32_t *ijk,
                                        double *pos) {
  auto s = tran->ptr->indexToWorld(vdb::Coord(ijk));
  pos[0] = s.x();
  pos[1] = s.y();
  pos[2] = s.z();
}

FLYWAVE_VDB_API void
voxel_transform_world_to_index_from_xyz(voxel_transform_t *tran, double *xyz,
                                        double *indexs) {
  auto s = tran->ptr->worldToIndex(vdb::Vec3d(xyz));
  indexs[0] = s.x();
  indexs[1] = s.y();
  indexs[2] = s.z();
}

FLYWAVE_VDB_API void
voxel_transform_world_to_index_cell_centered(voxel_transform_t *tran,
                                             double *xyz, int32_t *coords) {
  auto s = tran->ptr->worldToIndexCellCentered(vdb::Vec3d(xyz));
  coords[0] = s.x();
  coords[1] = s.y();
  coords[2] = s.z();
}

FLYWAVE_VDB_API void
voxel_transform_world_to_index_node_centered(voxel_transform_t *tran,
                                             double *xyz, int32_t *coords) {
  auto s = tran->ptr->worldToIndexNodeCentered(vdb::Vec3d(xyz));
  coords[0] = s.x();
  coords[1] = s.y();
  coords[2] = s.z();
}

FLYWAVE_VDB_API void
voxel_transform_index_to_world_from_coordbox(voxel_transform_t *tran,
                                             int32_t *ibbox, double *bbox) {
  vdb::CoordBBox b(vdb::Coord{ibbox[0], ibbox[1], ibbox[2]},
                   vdb::Coord{ibbox[3], ibbox[4], ibbox[5]});
  auto s = tran->ptr->indexToWorld(b);
  bbox[0] = s.min().x();
  bbox[1] = s.min().y();
  bbox[2] = s.min().z();

  bbox[3] = s.max().x();
  bbox[4] = s.max().y();
  bbox[5] = s.max().z();
}

FLYWAVE_VDB_API void
voxel_transform_index_to_world_from_bbox(voxel_transform_t *tran, double *bbox,
                                         double *tbbox) {
  vdb::BBoxd b(vdb::Vec3d{bbox[0], bbox[1], bbox[2]},
               vdb::Vec3d{bbox[3], bbox[4], bbox[5]});
  auto s = tran->ptr->indexToWorld(b);
  tbbox[0] = s.min().x();
  tbbox[1] = s.min().y();
  tbbox[2] = s.min().z();

  tbbox[3] = s.max().x();
  tbbox[4] = s.max().y();
  tbbox[5] = s.max().z();
}

FLYWAVE_VDB_API void
voxel_transform_world_to_index_from_bbox(voxel_transform_t *tran, double *bbox,
                                         double *tbbox) {
  vdb::BBoxd b(vdb::Vec3d{bbox[0], bbox[1], bbox[2]},
               vdb::Vec3d{bbox[3], bbox[4], bbox[5]});
  auto s = tran->ptr->worldToIndex(b);
  tbbox[0] = s.min().x();
  tbbox[1] = s.min().y();
  tbbox[2] = s.min().z();

  tbbox[3] = s.max().x();
  tbbox[4] = s.max().y();
  tbbox[5] = s.max().z();
}

FLYWAVE_VDB_API void voxel_transform_world_to_index_cell_centered_from_bbox(
    voxel_transform_t *tran, double *bbox, int32_t *cbbox) {
  vdb::BBoxd b(vdb::Vec3d{bbox[0], bbox[1], bbox[2]},
               vdb::Vec3d{bbox[3], bbox[4], bbox[5]});
  auto s = tran->ptr->worldToIndexCellCentered(b);
  cbbox[0] = s.min().x();
  cbbox[1] = s.min().y();
  cbbox[2] = s.min().z();

  cbbox[3] = s.max().x();
  cbbox[4] = s.max().y();
  cbbox[5] = s.max().z();
}

FLYWAVE_VDB_API void voxel_transform_world_to_index_node_centered_from_bbox(
    voxel_transform_t *tran, double *bbox, int32_t *cbbox) {
  vdb::BBoxd b(vdb::Vec3d{bbox[0], bbox[1], bbox[2]},
               vdb::Vec3d{bbox[3], bbox[4], bbox[5]});
  auto s = tran->ptr->worldToIndexNodeCentered(b);
  cbbox[0] = s.min().x();
  cbbox[1] = s.min().y();
  cbbox[2] = s.min().z();

  cbbox[3] = s.max().x();
  cbbox[4] = s.max().y();
  cbbox[5] = s.max().z();
}

FLYWAVE_VDB_API voxel_texture_mesh_t *
voxel_texture_mesh_create_from_triangles(voxel_io_triangle_t *tris, int count) {
  auto mesh = std::make_shared<flywave::texture_mesh>();
  mesh->load(reinterpret_cast<voxel_io_triangle *>(tris), count);
  return new voxel_texture_mesh_t{mesh};
}

FLYWAVE_VDB_API voxel_texture_mesh_t *voxel_texture_mesh_create() {
  return new voxel_texture_mesh_t{std::make_shared<flywave::texture_mesh>()};
}

FLYWAVE_VDB_API void voxel_texture_mesh_free(voxel_texture_mesh_t *vox) {
  delete vox;
}

FLYWAVE_VDB_API voxel_texture_mesh_t *
voxel_texture_mesh_duplicate(voxel_texture_mesh_t *vox) {
  return new voxel_texture_mesh_t{
      std::make_shared<flywave::texture_mesh>(*vox->mesh)};
}

FLYWAVE_VDB_API void
voxel_texture_mesh_get_triangles(voxel_texture_mesh_t *mesh,
                                 voxel_io_triangle_t **tris, int *count,
                                 int node) {
  *count = mesh->mesh->face.size();
  *tris = (voxel_io_triangle_t *)malloc(sizeof(voxel_io_triangle_t) * (*count));
  mesh->mesh->get_triangles(reinterpret_cast<voxel_io_triangle *>(tris), node);
}

FLYWAVE_VDB_API void voxel_texture_mesh_lock(voxel_texture_mesh_t *vox,
                                             _Bool *locked, int count) {
  vox->mesh->lock(reinterpret_cast<bool *>(locked), count);
}

FLYWAVE_VDB_API void voxel_texture_mesh_lock_border(voxel_texture_mesh_t *vox) {
  vox->mesh->lock_border();
}

FLYWAVE_VDB_API void
voxel_texture_mesh_unlock_border(voxel_texture_mesh_t *vox) {
  vox->mesh->unlock_border();
}

FLYWAVE_VDB_API void
voxel_texture_mesh_quadric_simplify_with_tex(voxel_texture_mesh_t *vox,
                                             uint32_t target) {
  vox->mesh->quadric_simplify_with_tex(target);
}

FLYWAVE_VDB_API void
voxel_texture_mesh_quadric_simplify(voxel_texture_mesh_t *vox,
                                    uint32_t target) {
  vox->mesh->quadric_simplify(target);
}

FLYWAVE_VDB_API voxel_mesh_builder_t *voxel_mesh_builder_create() {
  return new voxel_mesh_builder_t{std::make_shared<voxel_mesh_builder>()};
}

FLYWAVE_VDB_API void voxel_mesh_builder_free(voxel_mesh_builder_t *vox) {
  delete vox;
}

FLYWAVE_VDB_API void voxel_mesh_builder_set_name(voxel_mesh_builder_t *vox,
                                                 const char *name) {
  vox->ptr->set_name(name);
}

FLYWAVE_VDB_API void
voxel_mesh_builder_add_mesh_data(voxel_mesh_builder_t *vox,
                                 voxel_pixel_mesh_data_t *data) {
  vox->ptr->add_mesh_data(std::move(*data->data));
}

FLYWAVE_VDB_API void voxel_mesh_builder_add_material_data(
    voxel_mesh_builder_t *vox, voxel_pixel_material_data_t *data, int index) {
  vox->ptr->add_material_data(std::move(*data->data), index);
}

FLYWAVE_VDB_API void
voxel_mesh_builder_add_texture_data(voxel_mesh_builder_t *vox,
                                    voxel_pixel_texture_data_t *data,
                                    const char *name) {
  vox->ptr->add_texture(name, std::move(*data->data));
}

FLYWAVE_VDB_API _Bool
voxel_mesh_builder_texture_exist(voxel_mesh_builder_t *vox, const char *name) {
  return vox->ptr->texture_exist(name);
}

FLYWAVE_VDB_API voxel_texture_t *
voxel_mesh_builder_get_texture(voxel_mesh_builder_t *vox, const char *name) {
  return new voxel_texture_t{vox->ptr->get_textures().at(name)};
}

FLYWAVE_VDB_API _Bool
voxel_mesh_builder_material_exist(voxel_mesh_builder_t *vox, int index) {
  return vox->ptr->get_materials().find(index) !=
         vox->ptr->get_materials().end();
}

FLYWAVE_VDB_API voxel_material_t *
voxel_mesh_builder_get_material(voxel_mesh_builder_t *vox, int index) {
  return new voxel_material_t{vox->ptr->get_materials().at(index)};
}

FLYWAVE_VDB_API voxel_mesh_t *
voxel_mesh_builder_build_mesh(voxel_mesh_builder_t *vox) {
  return new voxel_mesh_t{vox->ptr->build_mesh(), vox->ptr->get_materials(),
                          vox->ptr->get_textures()};
}

#ifdef __cplusplus
}
#endif
