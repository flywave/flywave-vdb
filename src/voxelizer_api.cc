#include "voxelizer_api.h"
#include "texture_atlas.hh"
#include "texture_mesh.hh"
#include "transformer.hh"
#include "voxelizer_api_impl.hh"

#include "feature_meta_data.hh"
#include "grid_api_impl.hh"
#include "material_meta_data.hh"
#include "repacker.hh"
#include "voxel_mesh.hh"
#include "voxel_pixel_sampler.hh"

using namespace flywave;

#ifdef __cplusplus
extern "C" {
#endif

FLYWAVE_VDB_API void voxel_pixel_initialize() {
  register_feature_metadata_type();
  register_material_metadata_type();
  openvdb::initialize();
}

FLYWAVE_VDB_API void voxel_pixel_uninitialize() { openvdb::uninitialize(); }

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

FLYWAVE_VDB_API void voxel_pixel_clear_unuse_materials(voxel_pixel_t *vox) {
  vox->ptr->clear_unuse_materials();
}

FLYWAVE_VDB_API void voxel_pixel_clear_unuse_features(voxel_pixel_t *vox) {
  vox->ptr->clear_unuse_features();
}

FLYWAVE_VDB_API void voxel_pixel_clear_materials(voxel_pixel_t *vox) {
  vox->ptr->clear_materials();
}

FLYWAVE_VDB_API void voxel_pixel_clear_features(voxel_pixel_t *vox) {
  vox->ptr->clear_features();
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

FLYWAVE_VDB_API _Bool voxel_pixel_ray_tests(voxel_pixel_t *vox,
                                            double *ray_origin,
                                            double *ray_direction, double *p,
                                            size_t count) {
  std::vector<vdb::math::Ray<double>> rays;
  rays.resize(count);
  for (int i = 0; i < count; i++) {
    rays[i] = vdb::math::Ray<double>(
        vdb::Vec3d(ray_origin[(i * 3) + 0], ray_origin[(i * 3) + 1],
                   ray_origin[(i * 3) + 2]),
        vdb::Vec3d(ray_direction[(i * 3) + 0], ray_direction[(i * 3) + 1],
                   ray_direction[(i * 3) + 2]));
  }
  std::vector<vdb::Vec3d> query;
  query.resize(count);
  auto f = vox->ptr->ray_test(rays, query);
  memcpy(p, &query[0], count * sizeof(double) * 3);
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

FLYWAVE_VDB_API int64_t voxel_pixel_get_memory_size(voxel_pixel_t *vox) {
  return vox->ptr->get_memory_size();
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
  return new voxel_pixel_materials_t{vox->ptr->get_materials()};
}

FLYWAVE_VDB_API void voxel_pixel_set_materials(voxel_pixel_t *vox,
                                               voxel_pixel_materials_t *mtls) {
  vox->ptr->set_materials(mtls->mtls);
}

FLYWAVE_VDB_API voxel_pixel_features_t *
voxel_pixel_get_features(voxel_pixel_t *vox) {
  return new voxel_pixel_features_t{vox->ptr->get_features()};
}

FLYWAVE_VDB_API void voxel_pixel_set_features(voxel_pixel_t *vox,
                                              voxel_pixel_features_t *feats) {
  vox->ptr->set_features(feats->feats);
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

FLYWAVE_VDB_API voxel_texture_atlas_generator_t *
voxel_pixel_make_texture_atlas_generator(voxel_pixel_t *vox, double *mat,
                                         float pixel_pad) {
  auto repacker = std::make_shared<textute_repacker>(
      vox->ptr->get_voxel_grid(), vox->ptr->get_pixel_grid(), vdb::Mat4d(mat),
      pixel_pad);

  auto tgen = std::make_shared<texture_atlas_generator>();

  tgen->push_atlas(repacker);

  return new voxel_texture_atlas_generator_t{tgen};
}

FLYWAVE_VDB_API void
voxel_texture_atlas_generator_free(voxel_texture_atlas_generator_t *tag) {
  delete tag;
}

FLYWAVE_VDB_API void voxel_texture_atlas_generator_generate(
    voxel_texture_atlas_generator_t *tag, voxel_texture_mesh_t *tmesh,
    voxel_texture2d_t ***texs, size_t *texcount) {
  auto texvecs = tag->ptr->generate(*tmesh->mesh, *tmesh->mesh);

  *texcount = texvecs.size();
  *texs = (voxel_texture2d_t **)malloc(sizeof(voxel_texture2d_t *) *
                                       texvecs.size());
  for (int i = 0; i < texvecs.size(); i++) {
    *texs[i] = new voxel_texture2d_t{texvecs[i]};
  }
}

FLYWAVE_VDB_API voxel_texture2d_t *voxel_texture2d_create(uint32_t width,
                                                          uint32_t height) {
  return new voxel_texture2d_t{
      flywave::texture2d<flywave::vdb::math::Vec4<uint8_t>>::create(
          std::make_pair(width, height))};
}

FLYWAVE_VDB_API void voxel_texture2d_free(voxel_texture2d_t *tex) {
  delete tex;
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

FLYWAVE_VDB_API voxel_texture_mesh_t *
voxel_texture_mesh_create_from_mesh_datas(voxel_pixel_mesh_data_t **mdatas,
                                          int count) {
  auto mesh = std::make_shared<flywave::texture_mesh>();
  std::vector<std::shared_ptr<flywave::mesh_data>> datas;
  datas.resize(count);
  for (int i = 0; i < count; i++) {
    datas[i] = mdatas[i]->data;
  }
  mesh->load(datas);
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

FLYWAVE_VDB_API void voxel_texture_mesh_save(voxel_texture_mesh_t *vox,
                                             const char *path, uint32_t node) {
  vox->mesh->save(path, node);
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

FLYWAVE_VDB_API const char *
voxel_mesh_builder_get_name(voxel_mesh_builder_t *vox) {
  return vox->ptr->get_name().c_str();
}

FLYWAVE_VDB_API void
voxel_mesh_builder_add_mesh_data(voxel_mesh_builder_t *vox,
                                 voxel_pixel_mesh_data_t *data) {
  vox->ptr->add_mesh_data(data->data);
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

FLYWAVE_VDB_API _Bool
voxel_mesh_builder_material_exist(voxel_mesh_builder_t *vox, int index) {
  return vox->ptr->get_materials().find(index) !=
         vox->ptr->get_materials().end();
}

FLYWAVE_VDB_API voxel_mesh_t *
voxel_mesh_builder_build_mesh(voxel_mesh_builder_t *vox) {
  return new voxel_mesh_t{vox->ptr->build_mesh(), vox->ptr->get_materials(),
                          vox->ptr->get_textures()};
}

FLYWAVE_VDB_API voxel_texture_mesh_t *
voxel_mesh_builder_build_texture_mesh(voxel_mesh_builder_t *vox) {
  return new voxel_texture_mesh_t{vox->ptr->build_texture_mesh()};
}

FLYWAVE_VDB_API size_t
voxel_mesh_builder_get_material_size(voxel_mesh_builder_t *vox) {
  return vox->ptr->get_materials().size();
}

FLYWAVE_VDB_API size_t
voxel_mesh_builder_get_texture_size(voxel_mesh_builder_t *vox) {
  return vox->ptr->get_textures().size();
}

FLYWAVE_VDB_API size_t
voxel_mesh_builder_get_mesh_size(voxel_mesh_builder_t *vox) {
  return vox->ptr->get_mesh_datas().size();
}

FLYWAVE_VDB_API void voxel_mesh_free(voxel_mesh_t *m) { delete m; }

FLYWAVE_VDB_API _Bool voxel_mesh_empty(voxel_mesh_t *m) {
  return m->ptr->size() == 0;
}

FLYWAVE_VDB_API void voxel_mesh_clear(voxel_mesh_t *m) {
  m->ptr->clear();
  m->mtl_maps.clear();
  m->tex_maps.clear();
}

FLYWAVE_VDB_API void voxel_mesh_get_bounds(voxel_mesh_t *m, double *cbox) {
  auto box = m->ptr->get_bounds();
  cbox[0] = box.min().x();
  cbox[1] = box.min().y();
  cbox[2] = box.min().z();

  cbox[3] = box.max().x();
  cbox[4] = box.max().y();
  cbox[5] = box.max().z();
}

FLYWAVE_VDB_API voxel_pixel_t *
voxel_mesh_to_voxel_pixel(voxel_mesh_t *m, voxel_pixel_materials_t *mtls,
                          uint16_t local_feature, float precision,
                          voxel_clip_box_createor_t *creator, int32_t type,
                          double *matrix) {
  std::unique_ptr<voxel_mesh_adapter> stream =
      std::make_unique<voxel_mesh_adapter>(m->ptr, m->mtl_maps, m->tex_maps);
  auto stream_ptr = stream.get();

  mesh_adapter _mesh_adapter{std::move(stream)};

  stream_ptr->set_matrix(openvdb::Mat4d(matrix));

  material_merge_transfrom tmtl(
      mtls == nullptr ? std::vector<std::shared_ptr<material_data>>{}
                      : mtls->mtls);
  voxel_pixel_sampler sampler{_mesh_adapter, local_feature};
  std::shared_ptr<flywave::voxel_pixel> stff_pot =
      sampler.apply(precision, *creator->ptr, static_cast<sampler_type>(type),
                    tmtl, openvdb::Mat4d(matrix));

  if (mtls != nullptr)
    mtls->mtls = tmtl.materials();

  return new voxel_pixel_t{stff_pot};
}

static_assert(sizeof(c_material_data_t) == sizeof(material_data), "");

FLYWAVE_VDB_API voxel_pixel_material_data_t *
voxel_pixel_material_data_create(c_material_data_t data) {
  return new voxel_pixel_material_data_t{
      std::make_shared<flywave::material_data>(
          *(reinterpret_cast<material_data *>(&data)))};
}

FLYWAVE_VDB_API void
voxel_pixel_material_data_free(voxel_pixel_material_data_t *vox) {
  delete vox;
}

FLYWAVE_VDB_API c_material_data_t
voxel_pixel_material_data_get(voxel_pixel_material_data_t *vox) {
  return *(reinterpret_cast<c_material_data_t *>(vox->data.get()));
}

FLYWAVE_VDB_API void
voxel_pixel_material_data_set(voxel_pixel_material_data_t *vox,
                              c_material_data_t data) {
  vox->data = std::make_shared<flywave::material_data>(
      *(reinterpret_cast<material_data *>(&data)));
}

FLYWAVE_VDB_API void
voxel_pixel_add_material(voxel_pixel_t *vox,
                         voxel_pixel_material_data_t *mtls) {
  vox->ptr->add_material(mtls->data);
}

FLYWAVE_VDB_API void voxel_pixel_remove_material(voxel_pixel_t *vox,
                                                 uint8_t id) {
  vox->ptr->remove_material(static_cast<material_id_t>(id));
}

FLYWAVE_VDB_API bool voxel_pixel_has_material(voxel_pixel_t *vox, uint8_t id) {
  return vox->ptr->has_material(static_cast<material_id_t>(id));
}

FLYWAVE_VDB_API size_t voxel_pixel_materials_count(voxel_pixel_t *vox) {
  return vox->ptr->features_count();
}

FLYWAVE_VDB_API void voxel_pixel_add_feature(voxel_pixel_t *vox,
                                             voxel_pixel_feature_data_t *mtls) {
  vox->ptr->add_features(mtls->data);
}

FLYWAVE_VDB_API void voxel_pixel_remove_feature(voxel_pixel_t *vox,
                                                uint16_t id) {
  vox->ptr->remove_feature(static_cast<local_feature_id_t>(id));
}

FLYWAVE_VDB_API bool voxel_pixel_has_feature(voxel_pixel_t *vox, uint16_t id) {
  return vox->ptr->has_feature(static_cast<local_feature_id_t>(id));
}

FLYWAVE_VDB_API size_t voxel_pixel_features_count(voxel_pixel_t *vox) {
  return vox->ptr->materials_count();
}

FLYWAVE_VDB_API voxel_pixel_texture_data_t *
voxel_pixel_texture_data_create(c_texture_data_t t) {
  auto data = std::make_shared<texture_data>();
  data->width = t.width;
  data->height = t.height;
  data->format = static_cast<pixel_format>(t.format);
  size_t si = t.width * t.height;
  switch (data->format) {
  case pixel_format::RGB:
    si *= 3;
    break;
  case pixel_format::RGBA:
    si *= 4;
    break;
  default:
    break;
  }
  data->data.resize(si);
  memcpy(&data->data[0], t.data, si);
  return new voxel_pixel_texture_data_t{data};
}

FLYWAVE_VDB_API void voxel_pixel_materials_free(voxel_pixel_materials_t *mtls) {
  delete mtls;
}

FLYWAVE_VDB_API void voxel_pixel_features_free(voxel_pixel_features_t *feats) {
  delete feats;
}

FLYWAVE_VDB_API void
voxel_pixel_texture_data_free(voxel_pixel_texture_data_t *vox) {
  delete vox;
}

FLYWAVE_VDB_API c_texture_data_t
voxel_pixel_texture_data_get(voxel_pixel_texture_data_t *vox) {
  c_texture_data_t ret;
  ret.width = vox->data->width;
  ret.height = vox->data->height;
  ret.format = static_cast<int>(vox->data->format);
  ret.data = vox->data->data.data();
  return ret;
}

FLYWAVE_VDB_API void
voxel_pixel_texture_data_set(voxel_pixel_texture_data_t *vox,
                             c_texture_data_t t) {
  auto data = std::make_shared<texture_data>();
  data->width = t.width;
  data->height = t.height;
  data->format = static_cast<pixel_format>(t.format);
  size_t si = t.width * t.height;
  switch (data->format) {
  case pixel_format::RGB:
    si *= 3;
    break;
  case pixel_format::RGBA:
    si *= 4;
    break;
  default:
    break;
  }
  data->data.resize(si);
  memcpy(&data->data[0], t.data, si);
  vox->data = data;
}

FLYWAVE_VDB_API voxel_pixel_mesh_data_t *
voxel_pixel_mesh_data_create(c_mesh_data_t data) {
  auto mdata = std::make_shared<flywave::mesh_data>();
  mdata->vertices().resize(data.v_count);
  memcpy(&(mdata->vertices()[0]), data.vertices,
         data.v_count * 3 * sizeof(float));

  if (data.n_count > 0) {
    mdata->normals().resize(data.n_count);
    memcpy(&(mdata->normals()[0]), data.normals,
           data.n_count * 3 * sizeof(float));
  }

  if (data.t_count > 0) {
    mdata->texcoords().resize(data.t_count);
    memcpy(&(mdata->texcoords()[0]), data.texcoords,
           data.t_count * 2 * sizeof(float));
  }

  if (data.mtl_count > 0) {
    for (int i = 0; i < data.mtl_count; i++) {
      auto &mtl = data.mtl_map[i];
      if (mtl.f_count > 0) {
        std::vector<openvdb::Vec3I> faces;
        faces.resize(mtl.f_count);
        memcpy(&(faces[0]), mtl.faces, mtl.f_count * 3 * sizeof(uint32_t));
        mdata->add_mtl_faces(mtl.mtl, faces);
      }
      if (mtl.n_count > 0) {
        std::vector<openvdb::Vec3I> normals;
        normals.resize(mtl.n_count);
        memcpy(&(normals[0]), mtl.normals, mtl.n_count * 3 * sizeof(uint32_t));
        mdata->add_mtl_normals(mtl.mtl, normals);
      }
      if (mtl.t_count > 0) {
        std::vector<openvdb::Vec3I> texcoords;
        texcoords.resize(mtl.t_count);
        memcpy(&(texcoords[0]), mtl.texcoords,
               mtl.t_count * 3 * sizeof(uint32_t));
        mdata->add_mtl_texcoords(mtl.mtl, texcoords);
      }
    }
  }

  return new voxel_pixel_mesh_data_t{mdata};
}

FLYWAVE_VDB_API void voxel_pixel_mesh_data_free(voxel_pixel_mesh_data_t *vox) {
  delete vox;
}

FLYWAVE_VDB_API c_mesh_data_t
voxel_pixel_mesh_data_get(voxel_pixel_mesh_data_t *vox) {
  c_mesh_data_t ret;
  ret.v_count = vox->data->vertices().size();
  ret.vertices = reinterpret_cast<float *>(&(vox->data->vertices()[0]));

  if (vox->data->has_normal()) {
    ret.n_count = vox->data->normals().size();
    ret.normals = reinterpret_cast<float *>(&(vox->data->normals()[0]));
  }

  if (vox->data->has_texcoord()) {
    ret.t_count = vox->data->texcoords().size();
    ret.texcoords = reinterpret_cast<float *>(&(vox->data->texcoords()[0]));
  }

  if (vox->data->has_mtl_face()) {
    ret.mtl_map = (struct _c_mesh_data_mtl_t *)malloc(
        sizeof(struct _c_mesh_data_mtl_t) * vox->data->mtl_faces_map().size());
    ret.mtl_count = vox->data->mtl_faces_map().size();
    int index = 0;
    for (auto pair : vox->data->mtl_faces_map()) {
      auto mtl = pair.first;
      ret.mtl_map[index].mtl = mtl;

      ret.mtl_map[index].f_count = pair.second.size();
      ret.mtl_map[index].faces =
          reinterpret_cast<uint32_t *>(&(pair.second[0]));

      if (vox->data->mtl_normals_map().find(mtl) !=
          vox->data->mtl_normals_map().end()) {
        auto &nvs = vox->data->mtl_normals_map()[mtl];
        ret.mtl_map[index].n_count = nvs.size();
        ret.mtl_map[index].normals = reinterpret_cast<uint32_t *>(&(nvs[0]));
      }

      if (vox->data->mtl_texcoords_map().find(mtl) !=
          vox->data->mtl_texcoords_map().end()) {
        auto &texs = vox->data->mtl_texcoords_map()[mtl];

        ret.mtl_map[index].t_count = texs.size();
        ret.mtl_map[index].texcoords = reinterpret_cast<uint32_t *>(&(texs[0]));
      }

      index++;
    }
  }
  return ret;
}

FLYWAVE_VDB_API void voxel_pixel_mesh_data_set(voxel_pixel_mesh_data_t *vox,
                                               c_mesh_data_t data) {
  auto mdata = std::make_shared<flywave::mesh_data>();
  mdata->vertices().resize(data.v_count);
  memcpy(&(mdata->vertices()[0]), data.vertices,
         data.v_count * 3 * sizeof(float));

  if (data.n_count > 0) {
    mdata->normals().resize(data.n_count);
    memcpy(&(mdata->normals()[0]), data.normals,
           data.n_count * 3 * sizeof(float));
  }

  if (data.t_count > 0) {
    mdata->texcoords().resize(data.t_count);
    memcpy(&(mdata->texcoords()[0]), data.texcoords,
           data.t_count * 2 * sizeof(float));
  }

  if (data.mtl_count > 0) {
    for (int i = 0; i < data.mtl_count; i++) {
      auto &mtl = data.mtl_map[i];
      if (mtl.f_count > 0) {
        std::vector<openvdb::Vec3I> faces;
        faces.resize(mtl.f_count);
        memcpy(&(faces[0]), mtl.faces, mtl.f_count * 3 * sizeof(uint32_t));
        mdata->add_mtl_faces(mtl.mtl, faces);
      }
      if (mtl.n_count > 0) {
        std::vector<openvdb::Vec3I> normals;
        normals.resize(mtl.n_count);
        memcpy(&(normals[0]), mtl.normals, mtl.n_count * 3 * sizeof(uint32_t));
        mdata->add_mtl_normals(mtl.mtl, normals);
      }
      if (mtl.t_count > 0) {
        std::vector<openvdb::Vec3I> texcoords;
        texcoords.resize(mtl.t_count);
        memcpy(&(texcoords[0]), mtl.texcoords,
               mtl.t_count * 3 * sizeof(uint32_t));
        mdata->add_mtl_texcoords(mtl.mtl, texcoords);
      }
    }
  }
  vox->data = mdata;
}

extern _Bool clipBoxCreateor(void *ctx, vdb_float_grid_t *vertex,
                             voxel_transform_t *tran, double *sbox,
                             double *cbox);

class cgo_clip_box_createor : public clip_box_createor {
public:
  cgo_clip_box_createor(void *ctx) : _ctx(ctx) {}

  bool operator()(vertex_grid::Ptr vertex, vdb::math::Transform::Ptr resolution,
                  const vdb::BBoxd &sbox, vdb::BBoxd &cbox) override {
    double csbox[6]{sbox.min().x(), sbox.min().y(), sbox.min().z(),
                    sbox.max().x(), sbox.max().y(), sbox.max().z()};
    double ccbox[6];
    auto ret = clipBoxCreateor(
        _ctx,
        new vdb_float_grid_t{std::make_shared<flywave::vdb_float_grid>(vertex)},
        new voxel_transform_t{resolution}, csbox, ccbox);
    cbox = vdb::BBoxd(vdb::Vec3d{ccbox[0], ccbox[1], ccbox[2]},
                      vdb::Vec3d{ccbox[3], ccbox[4], ccbox[5]});
    return ret;
  }

  void *_ctx;
};

FLYWAVE_VDB_API voxel_clip_box_createor_t *
voxel_clip_box_createor_create(void *ctx) {
  return new voxel_clip_box_createor_t{
      std::make_shared<cgo_clip_box_createor>(ctx)};
}

FLYWAVE_VDB_API void
voxel_clip_box_createor_free(voxel_clip_box_createor_t *vox) {
  delete vox;
}

extern _Bool borderLockCheck(void *ctx, float *a);

class cgo_border_lock : public border_lock {
public:
  cgo_border_lock(void *ctx) : _ctx(ctx) {}

  bool is_need_lock(const vdb::math::Vec3<float> &a) override {
    return borderLockCheck(_ctx, const_cast<float *>(a.asPointer()));
  }

  void *_ctx;
};

FLYWAVE_VDB_API voxel_border_lock_t *voxel_border_lock_create(void *ctx) {
  return new voxel_border_lock_t{std::make_shared<cgo_border_lock>(ctx)};
}

FLYWAVE_VDB_API void voxel_border_lock_free(voxel_border_lock_t *vox) {
  delete vox;
}

extern _Bool filterTriangleValid(void *ctx, float *a, float *b, float *c);

class cgo_filter_triangle : public filter_triangle {
public:
  cgo_filter_triangle(void *ctx) : _ctx(ctx) {}

  bool valid(const vdb::math::Vec3<float> &a, const vdb::math::Vec3<float> &b,
             const vdb::math::Vec3<float> &c) override {
    return filterTriangleValid(_ctx, const_cast<float *>(a.asPointer()),
                               const_cast<float *>(b.asPointer()),
                               const_cast<float *>(c.asPointer()));
  }

  void *_ctx;
};

FLYWAVE_VDB_API voxel_filter_triangle_t *
voxel_filter_triangle_create(void *ctx) {
  return new voxel_filter_triangle_t{
      std::make_shared<cgo_filter_triangle>(ctx)};
}

FLYWAVE_VDB_API void voxel_filter_triangle_free(voxel_filter_triangle_t *vox) {
  delete vox;
}

FLYWAVE_VDB_API voxel_pixel_feature_data_t *
voxel_pixel_feature_data_create(c_feature_data_t t) {
  auto data = std::make_shared<feature_data>();
  data->_feature_id = t.local;
  data->_local_feature_id = t.global;
  size_t si = t.size;
  data->data.resize(si);
  memcpy(&data->data[0], t.data, si);
  return new voxel_pixel_feature_data_t{data};
}

FLYWAVE_VDB_API void
voxel_pixel_feature_data_free(voxel_pixel_feature_data_t *vox) {
  delete vox;
}

FLYWAVE_VDB_API c_feature_data_t
voxel_pixel_feature_data_get(voxel_pixel_feature_data_t *vox) {
  c_feature_data_t ret;
  ret.global = vox->data->_feature_id;
  ret.local = vox->data->_local_feature_id;
  ret.size = static_cast<int>(vox->data->data.size());
  ret.data = const_cast<uint8_t *>(
      reinterpret_cast<const uint8_t *>(vox->data->data.data()));
  return ret;
}

FLYWAVE_VDB_API void
voxel_pixel_feature_data_set(voxel_pixel_feature_data_t *vox,
                             c_feature_data_t cdata) {
  auto data = std::make_shared<feature_data>();
  data->_feature_id = cdata.global;
  data->_local_feature_id = cdata.local;
  size_t si = cdata.size;
  data->data.resize(si);
  memcpy(&data->data[0], cdata.data, si);
  vox->data = data;
}

extern _Bool isAffine(void *ctx);

extern _Bool isIdentity(void *ctx);

extern void gridTransform(void *ctx, double *pt, double *out);

extern void gridInvTransform(void *ctx, double *pt, double *out);

bool cgo_grid_transform::isAffine() const { return ::isAffine(ctx); }

bool cgo_grid_transform::isIdentity() const { return ::isIdentity(ctx); }

openvdb::Vec3R cgo_grid_transform::transform(const openvdb::Vec3R &pos) const {
  openvdb::Vec3R out;
  ::gridTransform(ctx, const_cast<double *>(pos.asPointer()), out.asPointer());
  return out;
}

openvdb::Vec3R
cgo_grid_transform::invTransform(const openvdb::Vec3R &pos) const {
  openvdb::Vec3R out;
  ::gridInvTransform(ctx, const_cast<double *>(pos.asPointer()),
                     out.asPointer());
  return out;
}

FLYWAVE_VDB_API voxel_grid_transform_t *voxel_grid_transform_create(void *ctx) {
  return new voxel_grid_transform_t{std::make_shared<cgo_grid_transform>(ctx)};
}

FLYWAVE_VDB_API void voxel_grid_transform_free(voxel_grid_transform_t *vox) {
  delete vox;
}

#ifdef __cplusplus
}
#endif
