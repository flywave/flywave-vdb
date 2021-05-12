#include "feature_meta_data.hh"

namespace flywave {

void register_feature_metadata_type() {
  openvdb::FeatureMetadata::registerType();
}

} // namespace flywave
