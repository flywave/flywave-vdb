#pragma once

#include "mesh_adapter.hh"
#include "texture2d.hh"
#include "voxel_pixel.hh"
#include "xparam.hh"

#include <openvdb/Types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct voxel_io_triangle;

#ifdef __cplusplus
}
#endif

namespace flywave {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

class textute_repacker {
  class impl;

public:
  textute_repacker(vertex_grid::Ptr cgrid, pixel_grid::Ptr, vdb::Mat4d mat,
                   float tpad);
  ~textute_repacker();

  texture2d<vdb::math::Vec4<uint8_t>>::Ptr
  extract(const fmesh_tri_patch &tri,
          texture2d<vdb::math::Vec4<uint8_t>>::Ptr img) const;

  size_t pixel_size() const;

private:
  std::unique_ptr<impl> _query;
  vertex_grid::Ptr _grid;
  float _pixel_pad;
  vdb::Mat4d _mat;
};

class border_lock {
public:
  virtual bool is_need_lock(const vdb::math::Vec3<float> &a) = 0;
};

class filter_triangle {
public:
  virtual bool valid(const vdb::math::Vec3<float> &a,
                     const vdb::math::Vec3<float> &,
                     const vdb::math::Vec3<float> &) = 0;
};

class triangle_repacker {
public:
  triangle_repacker(vertex_grid::Ptr vgrid) : _grid(vgrid) {}

  void build_poly(std::vector<vertext_type> &points,
                  std::vector<triangle_type> &tri, std::vector<quad_type> &a,
                  double isovalue = 0.0, double adapter = 0.01);

private:
  vertex_grid::Ptr _grid;
};

void make_triangles(std::vector<struct voxel_io_triangle> &vtriangles,
                    voxel_pixel &pot, const vdb::Mat4d &mat, size_t text_offset,
                    size_t mtl_offset, std::shared_ptr<border_lock>,
                    std::shared_ptr<filter_triangle> filter, double fquality,
                    double isovalue = 0.0, double adapter = 0.01);

void make_triangles(std::vector<struct voxel_io_triangle> &vtriangles,
                    voxel_pixel &pot, const vdb::Mat4d &mat, size_t text_offset,
                    size_t mtl_offset, double fquality, double isovalue = 0.0,
                    double adapter = 0.01);

} // namespace flywave
