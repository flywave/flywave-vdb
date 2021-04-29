#pragma once

#include "voxel_pixel.hh"

#ifdef __cplusplus
extern "C" {
#endif

struct _voxel_pixel_t {
  flywave::voxel_pixel *ptr;
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
