#pragma once

#include "mesh_adapter.hh"
#include "texture2d.hh"
#include "voxel_pot.hh"
#include "xparam.hh"

#include <openvdb/Types.h>

namespace flywave {
namespace voxelize {

class textute_foundry {
  class impl;

public:
  textute_foundry(vertex_grid::Ptr cgrid, pixel_grid::Ptr, float tquality,
                  float tpad);

  texture2d<openvdb::OPENVDB_VERSION_NAME::math::Vec4<uint8_t>>::Ptr
  extract(const fmesh_tri_patch &tri,
          texture2d<openvdb::OPENVDB_VERSION_NAME::math::Vec4<uint8_t>>::Ptr
              img) const;

  size_t pixel_size() const;

  float texture_quality() { return _texture_quality; }

private:
  std::unique_ptr<impl> _query;
  vertex_grid::Ptr _grid;
  float _texture_quality;
  float _pixel_pad;

public:
  ~textute_foundry();
};

struct seam_repair {
  using seam_tree = openvdb::tree::Tree4<uint32_t, 5, 4, 3>::Type;
  seam_tree::Ptr _seam_index_tree;
  std::shared_ptr<std::vector<vertext_type>> _seam_vertexs;
};

struct seam_box_setting {
  openvdb::OPENVDB_VERSION_NAME::math::BBox<double> _box;
  openvdb::OPENVDB_VERSION_NAME::math::Transform::ConstPtr _transform;
  openvdb::Vec3d _up;
  openvdb::Vec3d _left;
};

class triangle_foundry {
public:
  triangle_foundry(vertex_grid::Ptr vgrid) : _grid(vgrid) {}

  triangle_foundry(vertex_grid::Ptr vgrid, std::shared_ptr<seam_repair> repair)
      : _grid(vgrid), _seam_repair(repair) {}

  void make_mesh(std::vector<vertext_type> &points,
                 std::vector<triangle_type> &tri, std::vector<quad_type> &quads,
                 double isovalue = 0.0, double adapter = 0.01);

  std::shared_ptr<seam_repair> make_mesh_mark_seam(
      const seam_box_setting &mbox, std::vector<vertext_type> &points,
      std::vector<triangle_type> &tri, std::vector<quad_type> &a,
      double isovalue = 0.0, double adapter = 0.01);

private:
  vertex_grid::Ptr _grid;
  std::shared_ptr<seam_repair> _seam_repair;
};

template <typename Param> class packed_mesh : public Param {
public:
  packed_mesh(typename Param::mesh_type &mesh,
              std::shared_ptr<textute_foundry> tfoundry)
      : Param(mesh, tfoundry->texture_quality()),
        _tfoundry(std::move(tfoundry)) {}

public:
  texture2d<openvdb::math::Vec4<uint8_t>>::ptr pack_texture() {
    auto img_size = Param::image_size();
    if (img_size.x == 0 || img_size.y == 0)
      return nullptr;

    texture2d<openvdb::math::Vec4<uint8_t>>::ptr _texture =
        texture2d<openvdb::math::Vec4<uint8_t>>::create(
            {img_size.x, img_size.y});

    _texture->fill('\0');
    int count = 0;
    for (auto fptr : Param::mesh().face) {
      auto image = _tfoundry->extract(
          fmesh_tri_patch{
              vertext_type(fptr.P(0)[0], fptr.P(0)[1], fptr.P(0)[2]),
              vertext_type(fptr.P(1)[0], fptr.P(1)[1], fptr.P(1)[2]),
              vertext_type(fptr.P(2)[0], fptr.P(2)[1], fptr.P(2)[2]),
              uv_type(fptr.WT(0).U(), fptr.WT(0).V()),
              uv_type(fptr.WT(1).U(), fptr.WT(1).V()),
              uv_type(fptr.WT(2).U(), fptr.WT(2).V())},
          _texture);
    }
    double e = 1.0 / (img_size.x - 1);
    double e1 = 1.0 / (img_size.y - 1);
    for (auto fptr : Param::mesh().face) {
      fptr.V(0)->T().U() = fptr.WT(0).U() * e;
      fptr.V(0)->T().V() = fptr.WT(0).V() * e1;

      fptr.V(1)->T().U() = fptr.WT(1).U() * e;
      fptr.V(1)->T().V() = fptr.WT(1).V() * e1;

      fptr.V(2)->T().U() = fptr.WT(2).U() * e;
      fptr.V(2)->T().V() = fptr.WT(2).V() * e1;
    }
    return _texture;
  }

private:
  std::shared_ptr<textute_foundry> _tfoundry;
};

} // namespace voxelize
} // namespace flywave
