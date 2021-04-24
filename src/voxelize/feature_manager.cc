#pragma once
#include <flywave/core/io_file_pickle.hh>
#include <flywave/voxelize/feature_manager.hh>

namespace flywave {
namespace voxelize {

feature_descriptor::feature_descriptor(globe_feature_id_t gid,
                                       const bbox3f &box,
                                       const sstring &geojson)
    : _geojson{geojson} {
  _feature.id = gid;
  _feature.box = box;
  _feature.type = 0;
}

feature_descriptor::feature_descriptor(globe_feature_id_t gid,
                                       const sphere3f &sphere,
                                       const sstring &geojson)
    : _geojson{geojson} {
  _feature.id = gid;
  _feature.sphere = sphere;
  _feature.type = 1;
}

future<> feature_manager::write(random_access_writer &writer) {
  return do_with(local_feature_id_t{local_feature_id_t(_feature_pairs.size())},
                 [&writer, this](auto &size) {
                   return pickle::write(writer, size);
                 })
      .then([&writer, this] {
        return do_for_each(_feature_pairs, [&writer](auto &pair) {
          return pickle::write(writer, pair);
        });
      })
      .then([&writer, this] {
        return do_with(
            local_feature_id_t(_feature_descriptors.size()),
            [&writer, this](local_feature_id_t &size) {
              return pickle::write(writer, size).then([&writer, this] {
                return do_for_each(_feature_descriptors, [&writer](auto &pair) {
                  return pickle::write(writer, pair.second);
                });
              });
            });
      });
}

future<> feature_manager::read(random_access_reader &reader) {
  return do_with(local_feature_id_t{0}, local_feature_id_t{0},
                 [this, &reader](local_feature_id_t &size,
                                 local_feature_id_t &fsize) {
                   return pickle::parse(reader, size)
                       .then([this, &reader, &size] {
                         _feature_pairs = std::vector<feature_pair>(size);
                         return do_for_each(
                             _feature_pairs, [&reader](auto &pair) {
                               return pickle::parse(reader, pair);
                             });
                       })
                       .then([this, &reader, &fsize] {
                         return pickle::parse(reader, fsize);
                       })
                       .then([this, &reader, &fsize] {
                         return do_with(
                             feature_descriptor(),
                             make_number_range<uint32_t>(0u, fsize),
                             [&reader, this](auto &desc, auto &rang) {
                               return do_for_each(
                                   rang.begin(), rang.end(),
                                   [this, &reader, &desc](uint32_t block) {
                                     return pickle::parse(reader, desc)
                                         .then([this, &desc] {
                                           _feature_descriptors.emplace(
                                               desc.uid(), std::move(desc));
                                         });
                                   });
                             });

                       });
                 })
      .then([this] { initlize(std::move(_feature_pairs)); });
}

void feature_manager::initlize(std::vector<feature_pair> pair) {
  _feature_pairs = std::move(pair);
  for (auto &pair : _feature_pairs) {
    _globe_to_local_index.emplace(pair._globe_feature_id,
                                  pair._local_feature_id);

    _local_to_globe_index.emplace(pair._local_feature_id,
                                  pair._globe_feature_id);

    _local_feature_id_start =
        std::max(_local_feature_id_start, pair._local_feature_id);
  }
}

local_feature_id_t feature_manager::req_local_feature_id(globe_feature_id_t g) {
  auto it = _globe_to_local_index.find(g);
  if (it != _globe_to_local_index.end())
    return it->second;

  auto new_local_id = ++_local_feature_id_start;
  _globe_to_local_index.emplace(g, new_local_id);
  _local_to_globe_index.emplace(new_local_id, g);
  _feature_pairs.emplace_back(feature_pair{g, new_local_id});

  return new_local_id;
}

bool feature_manager::to_globe_feature_id(const local_feature_id_t &in,
                                          globe_feature_id_t &out) {
  auto iter = _local_to_globe_index.find(in);
  if (iter == _local_to_globe_index.end()) {
    return false;
  }
  out = iter->second;
  return true;
}

} // namespace voxelize
} // namespace flywave
