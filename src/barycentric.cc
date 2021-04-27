#include "barycentric.hh"

namespace flywave {

openvdb::Vec3d
bary_convert::bary_convert_impl::uv2bary(const openvdb::Vec2d &uv) {
  openvdb::Vec3d bary;
  double b0;

  b0 = (kb.x() - ka.x()) * (kc.y() - ka.y()) -
       (kc.x() - ka.x()) * (kb.y() - ka.y());

  bary.x() = ((kb.x() - uv.x()) * (kc.y() - uv.y()) -
              (kc.x() - uv.x()) * (kb.y() - uv.y())) /
             b0;
  bary.y() = ((kc.x() - uv.x()) * (ka.y() - uv.y()) -
              (ka.x() - uv.x()) * (kc.y() - uv.y())) /
             b0;
  bary.z() = ((ka.x() - uv.x()) * (kb.y() - uv.y()) -
              (kb.x() - uv.x()) * (ka.y() - uv.y())) /
             b0;

  return bary;
}

openvdb::Vec3d
bary_convert::bary_convert_impl::pos2bary(const openvdb::Vec3d &pos) {
  bary_convert_impl t;
  openvdb::Vec3d bary;
  if ((fabs(nrm.x()) > fabs(nrm.y())) && (fabs(nrm.x()) > fabs(nrm.z()))) {
    t.ka = openvdb::Vec2d(a.y(), a.z());
    t.kb = openvdb::Vec2d(b.y(), b.z());
    t.kc = openvdb::Vec2d(c.y(), c.z());
    bary = t.uv2bary(openvdb::Vec2d(pos.y(), pos.z()));
  } else if ((fabs(nrm.y()) > fabs(nrm.x())) &&
             (fabs(nrm.y()) > fabs(nrm.z()))) {
    t.ka = openvdb::Vec2d(a.x(), a.z());
    t.kb = openvdb::Vec2d(b.x(), b.z());
    t.kc = openvdb::Vec2d(c.x(), c.z());
    bary = t.uv2bary(openvdb::Vec2d(pos.x(), pos.z()));
  } else {
    t.ka = openvdb::Vec2d(a.x(), a.y());
    t.kb = openvdb::Vec2d(b.x(), b.y());
    t.kc = openvdb::Vec2d(c.x(), c.y());
    bary = t.uv2bary(openvdb::Vec2d(pos.x(), pos.y()));
  }
  return bary;
}

openvdb::Vec3d
bary_convert::bary_convert_impl::bary2pos(const openvdb::Vec3d &bary) {
  return openvdb::Vec3d(a.x() * bary.x() + b.x() * bary.y() + c.x() * bary.z(),
                        a.y() * bary.x() + b.y() * bary.y() + c.y() * bary.z(),
                        a.z() * bary.x() + b.z() * bary.y() + c.z() * bary.z());
}

openvdb::Vec2d
bary_convert::bary_convert_impl::bary2uv(const openvdb::Vec3d &bary) {
  return openvdb::Vec2d(
      ka.x() * bary.x() + kb.x() * bary.y() + kc.x() * bary.z(),
      ka.y() * bary.x() + kb.y() * bary.y() + kc.y() * bary.z());
}

} // namespace flywave
