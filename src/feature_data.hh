#pragma once

#include "types.hh"

#include <openvdb/openvdb.h>

#include <unordered_map>
#include <vector>

namespace flywave {

struct feature_data {
  globe_feature_id_t _feature_id = -1;
  std::string data;

  bool operator==(const feature_data &p) const;

  explicit operator bool() const {
    return _feature_id != globe_feature_id_t(-1);
  }
};

} // namespace flywave

std::ostream &operator<<(std::ostream &, const flywave::feature_data &);

namespace openvdb {
namespace OPENVDB_VERSION_NAME {

template <> inline const char *typeNameAsString<flywave::feature_data>() {
  return "feature";
}

template <> inline flywave::feature_data zeroVal<flywave::feature_data>() {
  return flywave::feature_data();
}
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb
