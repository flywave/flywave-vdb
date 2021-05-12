#pragma once

#include "material_data.hh"

#include <openvdb/openvdb.h>

#include <unordered_map>
#include <vector>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {

using MaterialMetadata = TypedMetadata<flywave::material_data>;

template <> inline std::string MaterialMetadata::str() const {
  return "material : " + std::to_string(mValue._material_id);
}

} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

namespace flywave {
void register_material_metadata_type();
}
