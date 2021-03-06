#pragma once

#include <openvdb/util/Util.h>

namespace flywave {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

template <typename AccessorT>
inline bool seach_vertex_value(const AccessorT &accessor, vdb::math::Coord ijk,
                               typename AccessorT::ValueType &value) {
  if (accessor.isValueOn(ijk)) {
    value = accessor.getValue(ijk);
    return true;
  }
  for (int32_t i = 0; i < 26; ++i) {
    auto nijk = ijk + openvdb::util::COORD_OFFSETS[i];
    if (accessor.isValueOn(nijk)) {
      value = accessor.getValue(nijk);
      return true;
    }
  }
  return false;
}

} // namespace flywave
