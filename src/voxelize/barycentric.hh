#pragma once

#include <flywave/math/triangle.hh>

namespace flywave {
namespace voxelize {

class bary_convert {
  struct bary_convert_impl {
    vector3<double> a, b, c;    // verts
    vector2<double> ka, kb, kc; // coords in kudzu space

    vector3<double> nrm; // triangle norm

    // barycentric stuff
    vector3<double> uv2bary(const vector2<double> &uv);
    vector3<double> pos2bary(const vector3<double> &pos);
    vector3<double> bary2pos(const vector3<double> &bary);
    vector2<double> bary2uv(const vector3<double> &bary);
  };

public:
  bary_convert()=default;
  bary_convert& operator=(const bary_convert&)=default;
  bary_convert(const triangle3<double> &tri, const triangle2<double> &uv)  {
    _impl.a = tri[0];
    _impl.b = tri[1];
    _impl.c = tri[2];

    _impl.ka = uv[0];
    _impl.kb = uv[1];
    _impl.kc = uv[2];

    vector3<double> a, b;
    a = (tri[1] - tri[0]).normalized();
    b = (tri[2] - tri[1]).normalized();
    _impl.nrm = a.crossed(b).normalized();
  }

  vector3<double> uv2bary(const vector2<double> &uv) {
    return _impl.uv2bary(uv);
  }

  vector3<double> pos2bary(const vector3<double> &pos) {
    return _impl.pos2bary(pos);
  }

  vector3<double> bary2pos(const vector3<double> &bary) {
    return _impl.bary2pos(bary);
  }

  vector2<double> bary2uv(const vector3<double> &bary) {
    return _impl.bary2uv(bary);
  }

private:
  bary_convert_impl _impl;
};

} // namespace voxelize
} // namespace flywave
