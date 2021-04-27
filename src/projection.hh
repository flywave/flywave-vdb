#pragma once

#include "barycentric.hh"
#include "rasterize.hh"
#include "triangle.hh"

namespace flywave {

class triangle_projection {
public:
  triangle_projection(const fmesh_tri_patch &tri, double pad = 2)
      : box2(), _tri(tri) {

    box2.extend(_tri.tp1);
    box2.extend(_tri.tp2);
    box2.extend(_tri.tp3);

    _convert = std::make_unique<bary_convert>(
        triangle3<double>(_tri.p1, _tri.p2, _tri.p3),
        triangle2<double>(_tri.tp1, _tri.tp2, _tri.tp3));
  }

  std::vector<openvdb::Vec3d>
  to_voxels(std::vector<vdb::math::Vec2<int>> &uvcoord) const {
    std::vector<openvdb::Vec3d> coords;
    if (box2.width() == 0 && box2.height() == 0)
      return std::vector<openvdb::Vec3d>();

    for (double y = std::floor(box2.min.y()); y <= std::ceil(box2.max.y());
         y++) {
      for (double x = std::floor(box2.min.x()); x <= std::ceil(box2.max.x());
           x++) {
        if (barycentric_coords_are_valid(
                _convert->uv2bary(openvdb::Vec2d(x, y))) ||
            barycentric_coords_are_valid(
                _convert->uv2bary(openvdb::Vec2d(x + 1, y))) ||
            barycentric_coords_are_valid(
                _convert->uv2bary(openvdb::Vec2d(x - 1, y))) ||
            barycentric_coords_are_valid(
                _convert->uv2bary(openvdb::Vec2d(x, y + 1))) ||
            barycentric_coords_are_valid(
                _convert->uv2bary(openvdb::Vec2d(x, y - 1))) ||
            barycentric_coords_are_valid(
                _convert->uv2bary(openvdb::Vec2d(x - 1, y + 1))) ||
            barycentric_coords_are_valid(
                _convert->uv2bary(openvdb::Vec2d(x + 1, y + 1))) ||
            barycentric_coords_are_valid(
                _convert->uv2bary(openvdb::Vec2d(x - 1, y - 1))) ||
            barycentric_coords_are_valid(
                _convert->uv2bary(openvdb::Vec2d(x + 1, y - 1)))) {
          coords.emplace_back(uv_to_point(openvdb::Vec2d(x, y)));
          uvcoord.emplace_back(vdb::math::Vec2<int>(x, y));
        }
      }
    }
    return coords;
  }

  bool barycentric_coords_are_valid(
      const openvdb::Vec3d &p_barycentricCoords) const {
    return p_barycentricCoords.x() >= 0.0f && p_barycentricCoords.y() >= 0.0f &&
           p_barycentricCoords.x() + p_barycentricCoords.y() <= 1.05f;
  }

  openvdb::Vec2d point_to_uv(const openvdb::Vec3d &point) const {
    auto uv = _convert->bary2uv(_convert->pos2bary(point));
    return openvdb::Vec2d(uv.x(), uv.y());
  }

  openvdb::Vec3d uv_to_point(const openvdb::Vec2d &uv) const {
    return _convert->bary2pos(_convert->uv2bary(uv));
  }

  const bbox2<double> &to_box2() const { return box2; }

  std::pair<int, int> size() const {
    return std::pair<int, int>(
        std::ceil(box2.max.x()) - std::floor(box2.min.x()),
        std::ceil(box2.max.y()) - std::floor(box2.min.y()));
  }

private:
  approx_value<double> app_value;
  bbox2<double> box2;
  fmesh_tri_patch _tri;

  std::unique_ptr<bary_convert> _convert;
};

} // namespace flywave
