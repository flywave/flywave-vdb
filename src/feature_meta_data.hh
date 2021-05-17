#pragma once

#include "feature_data.hh"

#include <openvdb/openvdb.h>

#include <unordered_map>
#include <vector>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {

using FeatureMetadata = TypedMetadata<flywave::feature_data>;

template <> inline std::string FeatureMetadata::str() const {
  return "feature : " + std::to_string(mValue._feature_id);
}

template <> inline Index32 FeatureMetadata::size() const {
  return static_cast<Index32>(mValue.data.size()) +
         sizeof(flywave::globe_feature_id_t);
}

template <>
inline void FeatureMetadata::readValue(std::istream &is, Index32 size) {
  is.read(reinterpret_cast<char *>(&mValue._feature_id),
          sizeof(flywave::globe_feature_id_t));

  size_t str_size = size - sizeof(flywave::globe_feature_id_t);
  mValue.data.resize(str_size, '\0');
  is.read(&mValue.data[0], str_size);
}

template <> inline void FeatureMetadata::writeValue(std::ostream &os) const {
  os.write(reinterpret_cast<const char *>(&mValue._feature_id),
           sizeof(flywave::globe_feature_id_t));
  size_t str_size = this->size() - sizeof(flywave::globe_feature_id_t);
  os.write(reinterpret_cast<const char *>(&mValue.data[0]), str_size);
}

} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

namespace flywave {
void register_feature_metadata_type();
}
