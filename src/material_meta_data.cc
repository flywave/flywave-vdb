#include "material_meta_data.hh"

namespace flywave {

void register_material_metadata_type() {
  openvdb::MaterialMetadata::registerType();
}

} // namespace flywave
