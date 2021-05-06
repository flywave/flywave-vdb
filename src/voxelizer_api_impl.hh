#pragma once

#include "material_data.hh"
#include "mesh_data.hh"
#include "repacker.hh"
#include "texture_data.hh"
#include "voxel_mesh.hh"
#include "voxel_pixel.hh"
#include "voxel_mesh_builder.hh"

#ifdef __cplusplus
extern "C" {
#endif

struct _voxel_pixel_t {
  std::shared_ptr<flywave::voxel_pixel> ptr;
};

struct _textute_repacker_t {
  std::shared_ptr<flywave::textute_repacker> ptr;
};

struct _voxel_mesh_t {
  std::shared_ptr<flywave::voxel_mesh> ptr;
  std::unordered_map<int, std::shared_ptr<flywave::material>> mtl_maps;
  std::unordered_map<std::string, std::shared_ptr<flywave::texture>> tex_maps;
};

struct _voxel_texture_t {
  std::shared_ptr<flywave::texture> tex;
};

struct _voxel_material_t {
  std::shared_ptr<flywave::material> mtl;
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

struct _io_vertex_t {
  float v[3];
  uint8_t c[4];  // colors
  float t[2];    // texture
  bool b{false}; // border

  bool operator==(const _io_vertex_t &p) const {
    return v[0] == p.v[0] && v[1] == p.v[1] && v[2] == p.v[2];
  }
};

struct _io_triangle_t {
  _io_vertex_t vertices[3];
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
