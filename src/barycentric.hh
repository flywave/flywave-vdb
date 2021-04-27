#pragma once

#include "triangle.hh"

#include <openvdb/Types.h>

namespace flywave {

class bary_convert {
  struct bary_convert_impl {
    openvdb::Vec3d a, b, c;    // verts
    openvdb::Vec2d ka, kb, kc; // coords in kudzu space

    openvdb::Vec3d nrm; // triangle norm

    // barycentric stuff
    openvdb::Vec3d uv2bary(const openvdb::Vec2d &uv);
    openvdb::Vec3d pos2bary(const openvdb::Vec3d &pos);
    openvdb::Vec3d bary2pos(const openvdb::Vec3d &bary);
    openvdb::Vec2d bary2uv(const openvdb::Vec3d &bary);
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

    openvdb::Vec3d a, b;
    a = (tri[1] - tri[0]).unitSafe();
    b = (tri[2] - tri[1]).unitSafe();
    _impl.nrm = (a * b).unitSafe();
  }

  openvdb::Vec3d uv2bary(const openvdb::Vec2d &uv) { return _impl.uv2bary(uv); }

  openvdb::Vec3d pos2bary(const openvdb::Vec3d &pos) {
    return _impl.pos2bary(pos);
  }

  openvdb::Vec3d bary2pos(const openvdb::Vec3d &bary) {
    return _impl.bary2pos(bary);
  }

  openvdb::Vec2d bary2uv(const openvdb::Vec3d &bary) {
    return _impl.bary2uv(bary);
  }

private:
  bary_convert_impl _impl;
};

} // namespace flywave
