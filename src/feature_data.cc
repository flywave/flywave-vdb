#include "feature_data.hh"

namespace flywave {

bool feature_data::operator==(const feature_data &p) const {
  return _feature_id == p._feature_id && data == p.data;
}
} // namespace flywave

std::ostream &operator<<(std::ostream &os, const flywave::feature_data &m) {
  os << "Feature{ id : " << m._feature_id << ", data : " << m.data << "}";
  return os;
}
