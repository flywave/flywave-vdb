#pragma once

#include "triangle.hh"

namespace flywave {
namespace voxelize {

class bary_convert {
  struct bary_convert_impl {
    Eigen::Matrix<double, 3, 1> a, b, c;    // verts
    Eigen::Matrix<double, 2, 1> ka, kb, kc; // coords in kudzu space

    Eigen::Matrix<double, 3, 1> nrm; // triangle norm

    // barycentric stuff
    Eigen::Matrix<double, 3, 1> uv2bary(const Eigen::Matrix<double, 2, 1> &uv);
    Eigen::Matrix<double, 3, 1>
    pos2bary(const Eigen::Matrix<double, 3, 1> &pos);
    Eigen::Matrix<double, 3, 1>
    bary2pos(const Eigen::Matrix<double, 3, 1> &bary);
    Eigen::Matrix<double, 2, 1>
    bary2uv(const Eigen::Matrix<double, 3, 1> &bary);
  };

public:
  bary_convert() = default;
  bary_convert &operator=(const bary_convert &) = default;

  bary_convert(const triangle3<double> &tri, const triangle2<double> &uv) {
    _impl.a = tri[0];
    _impl.b = tri[1];
    _impl.c = tri[2];

    _impl.ka = uv[0];
    _impl.kb = uv[1];
    _impl.kc = uv[2];

    Eigen::Matrix<double, 3, 1> a, b;
    a = (tri[1] - tri[0]).normalized();
    b = (tri[2] - tri[1]).normalized();
    _impl.nrm = (a * b).normalized();
  }

  Eigen::Matrix<double, 3, 1> uv2bary(const Eigen::Matrix<double, 2, 1> &uv) {
    return _impl.uv2bary(uv);
  }

  Eigen::Matrix<double, 3, 1> pos2bary(const Eigen::Matrix<double, 3, 1> &pos) {
    return _impl.pos2bary(pos);
  }

  Eigen::Matrix<double, 3, 1>
  bary2pos(const Eigen::Matrix<double, 3, 1> &bary) {
    return _impl.bary2pos(bary);
  }

  Eigen::Matrix<double, 2, 1> bary2uv(const Eigen::Matrix<double, 3, 1> &bary) {
    return _impl.bary2uv(bary);
  }

private:
  bary_convert_impl _impl;
};

} // namespace voxelize
} // namespace flywave
