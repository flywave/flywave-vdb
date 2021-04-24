#pragma once
#include <flywave/core/io_file_pickle.hh>
#include <flywave/core/io_file_stream.hh>
#include <flywave/lm/io.hh>
#include <flywave/voxelize/types.hh>

namespace flywave {
namespace voxelize {
class feature_manager;

class feature_descriptor {
  friend class feature_manager;
  feature_descriptor() = default;

public:
  feature_descriptor(globe_feature_id_t gid, const bbox3f &box,
                     const sstring &geojson);

  feature_descriptor(globe_feature_id_t gid, const sphere3f &sphere,
                     const sstring &geojson);

  template <typename Describer> auto describe_type(Describer f) {
    return f(_feature.type, _feature.id, _feature.sphere, _feature.box,
             _geojson);
  }

  globe_feature_id_t uid() const { return _feature.id; }

  const sstring &geojson() const { return _geojson.value; }
  const lm::feature &lmfeature() const { return _feature; }

private:
  lm::feature _feature;
  pickle::disk_string<uint32_t> _geojson;
};

class feature_manager {
  struct feature_pair {
    globe_feature_id_t _globe_feature_id;
    local_feature_id_t _local_feature_id;

    template <typename Describer> auto describe_type(Describer f) {
      return f(_globe_feature_id, _local_feature_id);
    }
  };

public:
  feature_manager(std::vector<feature_pair> pair) { initlize(std::move(pair)); }

  feature_manager() = default;

  future<> write(random_access_writer &writer);

  future<> read(random_access_reader &reader);

  local_feature_id_t req_local_feature_id(globe_feature_id_t g);

  bool to_globe_feature_id(const local_feature_id_t &in,
                           globe_feature_id_t &out);

  void add_feature(feature_descriptor &&feature) {
    _feature_descriptors.emplace(feature.uid(), std::move(feature));
  }

  const std::map<globe_feature_id_t, feature_descriptor> &
  feature_descriptors() const {
    return _feature_descriptors;
  }

private:
  void initlize(std::vector<feature_pair> pair);

private:
  std::vector<feature_pair> _feature_pairs;
  std::map<globe_feature_id_t, feature_descriptor> _feature_descriptors;

  local_feature_id_t _local_feature_id_start = 0;

  std::unordered_map<globe_feature_id_t, local_feature_id_t>
      _globe_to_local_index;
  std::unordered_map<local_feature_id_t, globe_feature_id_t>
      _local_to_globe_index;
};

} // namespace voxelize
} // namespace flywave
