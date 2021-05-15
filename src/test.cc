#include "types.hh"
#include "voxel_pixel_sampler.hh"
#include "voxelizer_api.h"
#include "voxelizer_api_impl.hh"

#include <memory>

#ifdef __cplusplus
extern "C" {
#endif

_Bool isAffine(void *ctx) { return false; }

_Bool isIdentity(void *ctx) { return false; }

void gridTransform(void *ctx, double *pt, double *out) {}

void gridInvTransform(void *ctx, double *pt, double *out) {}

_Bool borderLockCheck(void *ctx, float *a) { return false; }

_Bool filterTriangleValid(void *ctx, float *a, float *b, float *c) {
  return false;
}

_Bool clipBoxCreateor(void *ctx, vdb_float_grid_t *vertex,
                      voxel_transform_t *tran, double *sbox, double *cbox) {
  return false;
}

_Bool vdbFloatGridVisiton(void *ctx, int32_t *coord, float val) {
  return false;
}

_Bool vdbPixelGridVisiton(void *ctx, int32_t *coord, vdb_pixel_t val) {
  return false;
}

_Bool vdbFloatGridVisitonIterator(void *ctx, vdb_float_grid_iterator_t *iter) {
  return false;
}

_Bool vdbPixelGridVisitonIterator(void *ctx, vdb_pixel_grid_iterator_t *iter) {
  return false;
}

#ifdef __cplusplus
}
#endif

using namespace flywave;

void build_quad(std::shared_ptr<mesh_data> md, openvdb::Vec3s offset,
                openvdb::Vec3s widthDir, openvdb::Vec3s lengthDir) {
  openvdb::Vec3s normal = lengthDir.cross(widthDir).unit();

  md->add_vertice(offset);
  md->add_texcoord(openvdb::Vec2s(0.0, 0.0));
  md->add_normal(normal);

  md->add_vertice(offset + lengthDir);
  md->add_texcoord(openvdb::Vec2s(0.0, 1.0));
  md->add_normal(normal);

  md->add_vertice(offset + lengthDir + widthDir);
  md->add_texcoord(openvdb::Vec2s(1.0, 1.0));
  md->add_normal(normal);

  md->add_vertice(offset + widthDir);
  md->add_texcoord(openvdb::Vec2s(1.0, 0.0));
  md->add_normal(normal);

  uint32_t baseIndex = md->vertices().size() - 4;

  md->add_mtl_face(-1, openvdb::Vec3I(baseIndex, baseIndex + 1, baseIndex + 2));
  md->add_mtl_normal(-1,
                     openvdb::Vec3I(baseIndex, baseIndex + 1, baseIndex + 2));
  md->add_mtl_texcoord(-1,
                       openvdb::Vec3I(baseIndex, baseIndex + 1, baseIndex + 2));

  md->add_mtl_face(-1, openvdb::Vec3I(baseIndex, baseIndex + 2, baseIndex + 3));
  md->add_mtl_normal(-1,
                     openvdb::Vec3I(baseIndex, baseIndex + 2, baseIndex + 3));
  md->add_mtl_texcoord(-1,
                       openvdb::Vec3I(baseIndex, baseIndex + 2, baseIndex + 3));
}

std::shared_ptr<mesh_data> build_box(float length, float width) {
  auto upDir = openvdb::Vec3s(0, length, 0);
  auto rightDir = openvdb::Vec3s(width, 0, 0);
  auto forwardDir = openvdb::Vec3s(0, 0, length);

  auto nearCorner = openvdb::Vec3s(0, 0, 0);
  auto farCorner = upDir + rightDir + forwardDir;

  std::shared_ptr<mesh_data> md = std::make_shared<mesh_data>();

  build_quad(md, nearCorner, forwardDir, rightDir);
  build_quad(md, nearCorner, rightDir, upDir);
  build_quad(md, nearCorner, upDir, forwardDir);

  build_quad(md, farCorner, -rightDir, -forwardDir);
  build_quad(md, farCorner, -upDir, -rightDir);
  build_quad(md, farCorner, -forwardDir, -upDir);

  return md;
}

class test_clip_box_createor : public clip_box_createor {
public:
  bool operator()(vertex_grid::Ptr vertex, vdb::math::Transform::Ptr resolution,
                  const vdb::BBoxd &sbox, vdb::BBoxd &cbox) {
    return false;
  }
};

int test_mesh_builder() {
  std::shared_ptr<flywave::voxel_mesh_builder> vmb =
      std::make_shared<flywave::voxel_mesh_builder>();
  auto md = build_box(2, 2);
  vmb->add_mesh_data(md);

  auto cbuilder = new voxel_mesh_builder_t{vmb};
  voxel_mesh_t *vmesh = voxel_mesh_builder_build_mesh(cbuilder);

  std::unordered_map<int, std::shared_ptr<flywave::material>> map;
  std::unordered_map<std::string, std::shared_ptr<texture>> tex_map;

  std::unique_ptr<voxel_mesh_adapter> stream =
      std::make_unique<voxel_mesh_adapter>(vmesh->ptr, map, tex_map);

  mesh_adapter _mesh_adapter{std::move(stream)};

  test_clip_box_createor createor;

  material_merge_transfrom tmtl({});
  voxel_pixel_sampler sampler{_mesh_adapter, 1};
  std::shared_ptr<flywave::voxel_pixel> stff_pot = sampler.apply(
      10, createor, sampler_type::level_set, tmtl, openvdb::Mat4d::identity());

  voxel_mesh_free(vmesh);
  delete cbuilder;
  return 0;
};

int main(int argc, char **argv) {
  voxel_pixel_initialize();

  test_mesh_builder();

  voxel_pixel_uninitialize();
  return 0;
};
