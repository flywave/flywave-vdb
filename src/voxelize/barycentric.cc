#include "barycentric.hh"

namespace flywave {
namespace voxelize {

Eigen::Matrix<double, 3, 1> bary_convert::bary_convert_impl::uv2bary(
    const Eigen::Matrix<double, 2, 1> &uv) {
  Eigen::Matrix<double, 3, 1> bary;
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

Eigen::Matrix<double, 3, 1> bary_convert::bary_convert_impl::pos2bary(
    const Eigen::Matrix<double, 3, 1> &pos) {
  bary_convert_impl t;
  Eigen::Matrix<double, 3, 1> bary;
  if ((fabs(nrm.x()) > fabs(nrm.y())) && (fabs(nrm.x()) > fabs(nrm.z()))) {
    t.ka = Eigen::Matrix<double, 2, 1>(a.y(), a.z());
    t.kb = Eigen::Matrix<double, 2, 1>(b.y(), b.z());
    t.kc = Eigen::Matrix<double, 2, 1>(c.y(), c.z());
    bary = t.uv2bary(Eigen::Matrix<double, 2, 1>(pos.y(), pos.z()));
  } else if ((fabs(nrm.y()) > fabs(nrm.x())) &&
             (fabs(nrm.y()) > fabs(nrm.z()))) {
    t.ka = Eigen::Matrix<double, 2, 1>(a.x(), a.z());
    t.kb = Eigen::Matrix<double, 2, 1>(b.x(), b.z());
    t.kc = Eigen::Matrix<double, 2, 1>(c.x(), c.z());
    bary = t.uv2bary(Eigen::Matrix<double, 2, 1>(pos.x(), pos.z()));
  } else {
    t.ka = Eigen::Matrix<double, 2, 1>(a.x(), a.y());
    t.kb = Eigen::Matrix<double, 2, 1>(b.x(), b.y());
    t.kc = Eigen::Matrix<double, 2, 1>(c.x(), c.y());
    bary = t.uv2bary(Eigen::Matrix<double, 2, 1>(pos.x(), pos.y()));
  }
  return bary;
}

Eigen::Matrix<double, 3, 1> bary_convert::bary_convert_impl::bary2pos(
    const Eigen::Matrix<double, 3, 1> &bary) {
  return Eigen::Matrix<double, 3, 1>(
      a.x() * bary.x() + b.x() * bary.y() + c.x() * bary.z(),
      a.y() * bary.x() + b.y() * bary.y() + c.y() * bary.z(),
      a.z() * bary.x() + b.z() * bary.y() + c.z() * bary.z());
}

Eigen::Matrix<double, 2, 1> bary_convert::bary_convert_impl::bary2uv(
    const Eigen::Matrix<double, 3, 1> &bary) {
  return Eigen::Matrix<double, 2, 1>(
      ka.x() * bary.x() + kb.x() * bary.y() + kc.x() * bary.z(),
      ka.y() * bary.x() + kb.y() * bary.y() + kc.y() * bary.z());
}

} // namespace voxelize
} // namespace flywave