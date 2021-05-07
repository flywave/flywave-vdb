#pragma once

#include "material_data.hh"
#include "mesh_data.hh"
#include "repacker.hh"
#include "texture_data.hh"
#include "voxel_mesh.hh"
#include "voxel_mesh_builder.hh"
#include "voxel_pixel.hh"

#ifdef __cplusplus
extern "C" {
#endif

namespace flywave {
class texture_atlas_generator;
class texture_mesh;
} // namespace flywave

struct _voxel_pixel_t {
  std::shared_ptr<flywave::voxel_pixel> ptr;
};

struct _voxel_texture_atlas_generator_t {
  std::shared_ptr<flywave::texture_atlas_generator> ptr;
};

struct _voxel_transform_t {
  flywave::vdb::math::Transform::Ptr ptr;
};

struct _voxel_texture_mesh_t {
  std::shared_ptr<flywave::texture_mesh> mesh;
};

struct _voxel_mesh_t {
  std::shared_ptr<flywave::voxel_mesh> ptr;
  std::unordered_map<int, std::shared_ptr<flywave::material>> mtl_maps;
  std::unordered_map<std::string, std::shared_ptr<flywave::texture>> tex_maps;
};

struct _voxel_pixel_material_data_t {
  std::shared_ptr<flywave::material_data> data;
};

struct _voxel_pixel_materials_t {
  std::vector<std::shared_ptr<flywave::material_data>> mtls;
};

struct _voxel_pixel_texture_data_t {
  std::shared_ptr<flywave::texture_data> data;
};

struct _voxel_pixel_mesh_data_t {
  std::shared_ptr<flywave::mesh_data> data;
};

struct _voxel_mesh_builder_t {
  std::shared_ptr<flywave::voxel_mesh_builder> ptr;
};

struct _voxel_border_lock_t {
  std::shared_ptr<flywave::border_lock> ptr;
};

struct _voxel_filter_triangle_t {
  std::shared_ptr<flywave::filter_triangle> ptr;
};

struct _voxel_texture2d_t {
  std::shared_ptr<flywave::texture2d<flywave::vdb::math::Vec4<uint8_t>>> ptr;
};

struct _voxel_clip_box_createor_t {
  std::shared_ptr<flywave::clip_box_createor> ptr;
};

struct voxel_io_vertex {
  float v[3];
  uint8_t c[4];  // colors
  float t[2];    // texture
  bool b{false}; // border

  bool operator==(const voxel_io_vertex &p) const {
    return v[0] == p.v[0] && v[1] == p.v[1] && v[2] == p.v[2];
  }
};

struct voxel_io_triangle {
  voxel_io_vertex vertices[3];
  uint32_t node;
  uint32_t tex;
  uint32_t mtl;
  uint64_t feature_id;

  bool is_degenerate() const {
    if (vertices[0] == vertices[1] || vertices[0] == vertices[2] ||
        vertices[1] == vertices[2])
      return true;
    return false;
  }
};

#ifdef __cplusplus
}
#endif
