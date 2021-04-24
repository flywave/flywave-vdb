
#include <flywave/core/io_file_pickle.hh>
#include <flywave/voxelize/voxel_pot.hh>

#include <flywave/vdb/storage/compression.hh>
#include <flywave/vdb/tools/clip.hh>
#include <flywave/vdb/tools/composite.hh>
#include <flywave/vdb/tools/ray_intersector.hh>

namespace flywave {
namespace voxelize {

voxel_pot::voxel_pot(vertex_grid::ptr vertex, pixel_grid::ptr pixel,
                     vdb::math::transform::ptr res)
    : _resolution(res), _vertex(vertex), _pixel(pixel) {
  _vertex->set_grid_class(vdb::GRID_LEVEL_SET);
  _vertex->set_transform(_resolution);
}

future<> voxel_pot::write(random_access_writer &writer) {
  return do_with(
      file_patch{_resolution, _vertex, _pixel},
      [this, &writer](file_patch &patch) {
        return patch._resolution->base_map()
            ->write(writer)
            .then([this, &writer] {
              return do_with(
                  uint16_t(_materials.size()), [this, &writer](auto &size) {
                    return pickle::write(writer, size).then([this, &writer] {
                      return do_for_each(_materials, [&writer](auto &material) {
                        return pickle::write(writer, *material);
                      });
                    });
                  });
            })
            .then([this, &writer] { return _feature_manager.write(writer); })
            .then([&patch, &writer] {
              vdb::storage::set_data_compression(
                  writer, vdb::storage::COMPRESS_ZIP |
                              vdb::storage::COMPRESS_ACTIVE_MASK);

              return patch._vertex->write_topology(writer)
                  .then([&patch, &writer] {
                    return patch._vertex->write_buffers(writer);
                  })
                  .then([&patch, &writer] {
                    vdb::storage::set_data_compression(
                        writer, vdb::storage::COMPRESS_ZIP
                        //  | vdb::storage::COMPRESS_ACTIVE_MASK
                    );
                    return patch._pixel->write_topology(writer).then(
                        [&patch, &writer] {
                          return patch._pixel->write_buffers(writer);
                        });
                  });
            });
      });
}

future<> voxel_pot::read(const file_path &file) {
  return io_virtual_file::open(file, os_file_flags::OS_FILE_READ)
      .then([this](io_virtual_file file) { return read(file); });
}

future<> voxel_pot::read(io_virtual_file file) {
  _file = file;
  auto fut = file.size();
  return fut
      .then([fi = file](uint64_t size) {
        return make_ready_future<file_random_access_reader>(
            file_random_access_reader{std::move(fi), size,
                                      default_priority_group(), 8192});
      })
      .then([this, file](file_random_access_reader reader) {
        return do_with(std::move(reader), [this, file](auto &reader) {
          return read(reader).then([&reader] { return reader.close(); });
        });
      });
}

future<> voxel_pot::read(random_access_reader &reader) {
  return do_with(
      file_patch{vdb::math::transform::create_linear_transform(1.0), _vertex,
                 _pixel},
      [&reader, this](file_patch &patch) {
        return patch._resolution->base_map()
            ->read(reader)
            .then([this, &reader] {
              return do_with(uint16_t{0}, [this, &reader](uint16_t &size) {
                return pickle::parse(reader, size).then([this, &reader, &size] {
                  _materials = std::vector<shared_ptr<material_data>>(size);
                  return do_for_each(_materials, [&reader](auto &material) {
                    material = make_shared<material_data>();
                    return pickle::parse(reader, *material);
                  });
                });
              });
            })
            .then([this, &reader] { return _feature_manager.read(reader); })
            .then([&patch, &reader] {
              vdb::storage::set_data_compression(
                  (random_access_reader &)reader,
                  vdb::storage::COMPRESS_ZIP |
                      vdb::storage::COMPRESS_ACTIVE_MASK);

              return patch._vertex->read_topology(reader).then(
                  [&patch, &reader] {
                    return patch._vertex->read_buffers(reader);
                  });
            })
            .then([&patch, &reader] {
              vdb::storage::set_data_compression((random_access_reader &)reader,
                                                 vdb::storage::COMPRESS_ZIP);
              return patch._pixel->read_topology(reader).then(
                  [&patch, &reader] {
                    return patch._pixel->read_buffers(reader);
                  });
            })
            .then([this, &patch] {
              _resolution = patch._resolution;
              patch._vertex->set_transform(_resolution);
              patch._vertex->set_grid_class(vdb::GRID_LEVEL_SET);
            });
      });
}

bool voxel_pot::ray_test(const flywave::extray3<double> &ray, vdb::vec3d &p) {
  if (_vertex->empty())
    return false;
  vdb::tools::level_set_ray_intersector<vertex_grid> vray(*_vertex);

  return vray.intersects_WS(ray, p);
}

void voxel_pot::clear_unuse_materials() {
  std::map<material_id_t, bool> mapping;
  for (auto pt : _materials) {
    mapping.emplace(pt->_material_id, true);
  }

  auto iter = pixel_grid()->tree().begin_value_on();
  while (iter) {
    mapping[iter.get_value()._data._material_id] = false;
    ++iter;
  }

  for (auto pt : mapping) {
    if (pt.second == false)
      _materials.erase(_materials.begin() + pt.first);
  }
}

void voxel_pot_intersection(voxel_pot &tpot, voxel_pot &spot) {
  vdb::tools::csg_intersection(tpot.voxel_grid()->tree(),
                               spot.voxel_grid()->tree(), true);
}

void voxel_pot_union(voxel_pot &tpot, voxel_pot &spot) {
  auto pix_size = tpot.voxel_grid()->active_voxel_count();
  // if (pix_size == 0)
  //   tpot.set_voxel_grid(spot.voxel_grid());
  // else
  vdb::tools::csg_union(tpot.voxel_grid()->tree(), spot.voxel_grid()->tree(),
                        true);
  tpot.pixel_grid()->tree().merge(spot.pixel_grid()->tree());

  auto iter = spot.voxel_grid()->tree().begin_value_on();
  auto paccess = tpot.pixel_grid()->get_accessor();
  auto vaccess = tpot.voxel_grid()->get_accessor();

  while (iter) {
    if (!vaccess.is_value_on(iter.get_coord())) {
      paccess.set_value_off(iter.get_coord());
    }

    ++iter;
  }

  vdb::tools::prune_level_set(tpot.voxel_grid()->tree());
}

void voxel_pot_difference(voxel_pot &tpot, voxel_pot &spot) {
  vdb::tools::csg_difference(tpot.voxel_grid()->tree(),
                             spot.voxel_grid()->tree(), true);
}

void voxel_pot::composite(voxel_pot &pot, const composite_type &type) {
  // merge_materials(pot);
  _resolution = pot._resolution;

  if (!pot.is_empty()) {
    switch (type) {
    case composite_type::op_union:
      voxel_pot_union(*this, pot);
      break;

    case composite_type::op_intersection:
      voxel_pot_intersection(*this, pot);
      break;

    case composite_type::op_difference:
      voxel_pot_difference(*this, pot);
      break;
    }
  }
  _vertex->set_transform(_resolution);
}

} // namespace voxelize
} // namespace flywave
