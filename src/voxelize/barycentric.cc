
#include <flywave/voxelize/barycentric.hh> 

namespace flywave {
namespace voxelize {

vector3<double>
bary_convert::bary_convert_impl::uv2bary(const vector2<double> &uv) {
  vector3<double> bary;
  double b0;

  b0 = (kb.x - ka.x) * (kc.y - ka.y) - (kc.x - ka.x) * (kb.y - ka.y);
  bary.x = ((kb.x - uv.x) * (kc.y - uv.y) - (kc.x - uv.x) * (kb.y - uv.y)) / b0;
  bary.y = ((kc.x - uv.x) * (ka.y - uv.y) - (ka.x - uv.x) * (kc.y - uv.y)) / b0;
  bary.z = ((ka.x - uv.x) * (kb.y - uv.y) - (kb.x - uv.x) * (ka.y - uv.y)) / b0;
  // printf("DBG: %f %f %f = %f\n", bary.x, bary.y, bary.z, bary.x + bary.y +
  // bary.z );

  return bary;
}

vector3<double>
bary_convert::bary_convert_impl::pos2bary(const vector3<double> &pos) {
  // project based on normal
  bary_convert_impl t;
  vector3<double> bary;
  if ((fabs(nrm.x) > fabs(nrm.y)) && (fabs(nrm.x) > fabs(nrm.z))) {
    // project along x
    t.ka = vector2<double>(a.y, a.z);
    t.kb = vector2<double>(b.y, b.z);
    t.kc = vector2<double>(c.y, c.z);
    bary = t.uv2bary(vector2<double>(pos.y, pos.z));
  } else if ((fabs(nrm.y) > fabs(nrm.x)) && (fabs(nrm.y) > fabs(nrm.z))) {
    // project along y
    t.ka = vector2<double>(a.x, a.z);
    t.kb = vector2<double>(b.x, b.z);
    t.kc = vector2<double>(c.x, c.z);
    bary = t.uv2bary(vector2<double>(pos.x, pos.z));
  } else {
    // project along z
    t.ka = vector2<double>(a.x, a.y);
    t.kb = vector2<double>(b.x, b.y);
    t.kc = vector2<double>(c.x, c.y);
    bary = t.uv2bary(vector2<double>(pos.x, pos.y));
  }
  return bary;
}

vector3<double>
bary_convert::bary_convert_impl::bary2pos(const vector3<double> &bary) {
  return vector3<double>(a.x * bary.x + b.x * bary.y + c.x * bary.z,
                         a.y * bary.x + b.y * bary.y + c.y * bary.z,
                         a.z * bary.x + b.z * bary.y + c.z * bary.z);
}

vector2<double>
bary_convert::bary_convert_impl::bary2uv(const vector3<double> &bary) {
  return vector2<double>(ka.x * bary.x + kb.x * bary.y + kc.x * bary.z,
                         ka.y * bary.x + kb.y * bary.y + kc.y * bary.z);
}

} // namespace voxelize
} // namespace flywave