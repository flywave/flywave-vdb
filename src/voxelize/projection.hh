#pragma once

#include <flywave/voxelize/barycentric.hh>
#include <flywave/voxelize/rasterize.hh>

namespace flywave {
namespace voxelize {

class triangle_projection {
public:
  triangle_projection(const fmesh_tri_patch &tri, double pad = 2) : _tri(tri) {

    box2.extend(_tri.tp1);
    box2.extend(_tri.tp2);
    box2.extend(_tri.tp3);

    _convert = std::make_unique<bary_convert>(
        triangle3<double>(_tri.p1, _tri.p2, _tri.p3),
        triangle2<double>(_tri.tp1, _tri.tp2, _tri.tp3));
  }

  std::vector<openvdb::Vec3d> to_voxels() const {
    std::vector<openvdb::Vec3d> coords;
    if (box2.width() == 0 && box2.height() == 0)
      return std::vector<openvdb::Vec3d>();

    for (double y = std::floor(box2.min.y); y <= std::ceil(box2.max.y); y++) {
      for (double x = std::floor(box2.min.x); x <= std::ceil(box2.max.x); x++) {
        if (BarycentricCoordsAreValid(_convert->uv2bary({x, y})) ||
            BarycentricCoordsAreValid(_convert->uv2bary({x + 1, y})) ||
            BarycentricCoordsAreValid(_convert->uv2bary({x - 1, y})) ||
            BarycentricCoordsAreValid(_convert->uv2bary({x, y + 1})) ||
            BarycentricCoordsAreValid(_convert->uv2bary({x, y - 1})) ||
            BarycentricCoordsAreValid(_convert->uv2bary({x - 1, y + 1})) ||
            BarycentricCoordsAreValid(_convert->uv2bary({x + 1, y + 1})) ||
            BarycentricCoordsAreValid(_convert->uv2bary({x - 1, y - 1})) ||
            BarycentricCoordsAreValid(_convert->uv2bary({x + 1, y - 1}))) {
          coords.emplace_back(uv_to_point(Eigen::Matrix<double, 2, 1>(x, y)));
        }
      }
    }
    return std::move(coords);
  }

  bool
  BarycentricCoordsAreValid(const Eigen::Matrix<double, 3, 1> &p_barycentricCoords) const {
    return p_barycentricCoords.x >= 0.0f && p_barycentricCoords.y >= 0.0f &&
           p_barycentricCoords.x + p_barycentricCoords.y <= 1.05f;
  }

  Eigen::Matrix<double, 2, 1> point_to_uv(const Eigen::Matrix<double, 3, 1> &point) const {
    auto uv = _convert->bary2uv(_convert->pos2bary(point));
    return Eigen::Matrix<double, 2, 1>(uv.x, uv.y);
  }

  Eigen::Matrix<double, 3, 1> uv_to_point(const Eigen::Matrix<double, 2, 1> &uv) const {
    return _convert->bary2pos(_convert->uv2bary(uv));
  }

  const bbox2<double> &to_box2() const { return box2; }

  vector2<int> size() const {
    return vector2<int>(std::ceil(box2.max.x) - std::floor(box2.min.x),
                        std::ceil(box2.max.y) - std::floor(box2.min.y));
  }

private:
  bool approx_inside() {}

private:
  approx_value<double> app_value;
  bbox2<double> box2;
  fmesh_tri_patch _tri;

  std::unique_ptr<bary_convert> _convert;
};
} // namespace voxelize
} // namespace flywave
