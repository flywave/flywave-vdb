#pragma once

#include "mesh_adapter.hh"
#include "texture2d.hh"
#include "voxel_pot.hh"
#include "xparam.hh"

#include <openvdb/Types.h>

namespace flywave {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

class textute_foundry {
  class impl;

public:
  textute_foundry(vertex_grid::Ptr cgrid, pixel_grid::Ptr, vdb::Mat4d mat,
                  float tpad);

  texture2d<vdb::math::Vec4<uint8_t>>::Ptr
  extract(const fmesh_tri_patch &tri,
          texture2d<vdb::math::Vec4<uint8_t>>::Ptr img) const;

  size_t pixel_size() const;

private:
  std::unique_ptr<impl> _query;
  vertex_grid::Ptr _grid;
  float _pixel_pad;
  vdb::Mat4d _mat;

public:
  ~textute_foundry();
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

class triangle_foundry {
public:
  triangle_foundry(vertex_grid::Ptr vgrid) : _grid(vgrid) {}

  void make_mesh_mark_seam(std::vector<vertext_type> &points,
                           std::vector<triangle_type> &tri,
                           std::vector<quad_type> &a, double isovalue = 0.0,
                           double adapter = 0.01);

private:
  vertex_grid::Ptr _grid;
};

} // namespace flywave
