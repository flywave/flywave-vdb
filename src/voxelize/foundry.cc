#include "foundry.hh"
#include "index.hh"
#include "projection.hh"
#include "resolution.hh"
#include "sampler.hh"
#include "voxel_pot.hh"

#include "tolerance.hh"

#include <openvdb/math/Transform.h>
#include <openvdb/tools/VolumeToMesh.h>

#include <wrap/io_trimesh/export_obj.h>

namespace flywave {
namespace voxelize {

class textute_foundry::impl {
public:
  using sampler_type = sampling_result<pixel_grid::ValueType>;

public:
  impl(vertex_grid::Ptr cgrid, pixel_grid::Ptr pgrid) {
    _query = make_near_voxels_index<vertex_grid, pixel_grid>(cgrid, pgrid);
  }

  std::vector<sampler_type> extract(std::vector<openvdb::Vec3d> coords) const {
    return _query->extract(std::move(coords));
  }

  static texture2d<vdb::math::Vec4<uint8_t>>::Ptr
  project_to_image(std::vector<sampler_type> points,
                   const triangle_projection &prj,
                   const std::vector<vdb::math::Vec2<int>> &coord,
                   texture2d<vdb::math::Vec4<uint8_t>>::Ptr);

  size_t pixel_size() const { return _query->pixel_count(); }

private:
  std::unique_ptr<triangle_range_query<pixel_grid>> _query;
};

textute_foundry::textute_foundry(vertex_grid::Ptr cgrid, pixel_grid::Ptr pgrid,
                                 openvdb::Mat4d mat, float pixel_pad)
    : _query(std::make_unique<impl>(cgrid, pgrid)), _grid(cgrid),
      _pixel_pad(pixel_pad), _mat(mat) {}

texture2d<vdb::math::Vec4<uint8_t>>::Ptr textute_foundry::extract(
    const fmesh_tri_patch &tri,
    texture2d<vdb::math::Vec4<uint8_t>>::Ptr texture) const {
  fmesh_tri_patch ptri = tri;
  ptri.p1 = _grid->transform().worldToIndex(tri.p1);
  ptri.p2 = _grid->transform().worldToIndex(tri.p2);
  ptri.p3 = _grid->transform().worldToIndex(tri.p3);

  triangle_projection proj(ptri, _pixel_pad);

  std::vector<vdb::math::Vec2<int>> outcoordl;
  auto q = proj.to_voxels(outcoordl);
  return impl::project_to_image(_query->extract(std::move(q)), proj,
                                std::move(outcoordl), texture);
}

size_t textute_foundry::pixel_size() const { return _query->pixel_size(); }

textute_foundry::~textute_foundry() = default;

texture2d<vdb::math::Vec4<uint8_t>>::Ptr
textute_foundry::impl::project_to_image(
    std::vector<sampler_type> points, const triangle_projection &prj,
    const std::vector<vdb::math::Vec2<int>> &coord,
    texture2d<vdb::math::Vec4<uint8_t>>::Ptr img) {

  auto h = img->height();

  for (int i = 0; i < points.size(); i++) {

    auto &uv = coord[i];
    auto &pt = points[i];
    uint64_t min_x = std::min(uint64_t(std::floor(uv.x())), img->width() - 1);
    uint64_t min_y = std::min(uint64_t(std::floor(h - uv.y() - 1)), h - 1);
    uint64_t max_x = std::min(uint64_t(std::ceil(uv.x())), img->width() - 1);
    uint64_t max_y = std::min(uint64_t(std::ceil(h - uv.y() - 1)), h - 1);

    img->color(min_x, min_y) = pt._value._data._color;
    img->color(max_x, max_y) = pt._value._data._color;
    img->color(min_x, max_y) = pt._value._data._color;
    img->color(max_x, min_y) = pt._value._data._color;
  }
  return img;
}

void triangle_foundry::make_mesh_mark_seam(std::vector<vertext_type> &points,
                                           std::vector<triangle_type> &tri,
                                           std::vector<quad_type> &quads,
                                           double isovalue, double adapter) {
  vdb::tools::volumeToMesh(*_grid, points, tri, quads, isovalue, adapter, true);
}

} // namespace voxelize
} // namespace flywave
