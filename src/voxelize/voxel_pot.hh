#pragma once
#include <flywave/voxelize/feature_manager.hh>
#include <flywave/voxelize/mesh_adapter.hh> 
#include <flywave/voxelize/trees.hh>

namespace flywave {
namespace voxelize {

class micronizer;
class voxel_pot;

class voxel_pot {
  friend class micronizer;

public:
  voxel_pot()
      : _vertex(vertex_grid::create(std::numeric_limits<float>::max())),
        _pixel(pixel_grid::create()),
        _resolution(vdb::math::transform::create_linear_transform()) {
    _vertex->set_transform(_resolution);
    _vertex->set_grid_class(vdb::GRID_LEVEL_SET);
  }

  voxel_pot(vdb::math::transform::ptr transfrom)
      : _vertex(vertex_grid::create(std::numeric_limits<float>::max())),
        _pixel(pixel_grid::create()), _resolution(transfrom) {
    _vertex->set_transform(_resolution);
    _vertex->set_grid_class(vdb::GRID_LEVEL_SET);
  }

  vdb::math::transform::ptr voxel_resolution() const { return _resolution; }

  vertex_grid::ptr voxel_grid() const { return _vertex; }

  pixel_grid::ptr pixel_grid() const { return _pixel; }

  void set_voxel_grid(vertex_grid::ptr ptr) {
    _vertex = ptr;
    _vertex->set_transform(_resolution);
  }

  void set_pixel_grid(pixel_grid::ptr ptr) { _pixel = ptr; }

  bool is_empty() { return _vertex == nullptr; }

  std::vector<shared_ptr<material_data>> materials() {
    return std::move(_materials);
  }

  void set_materials(std::vector<shared_ptr<material_data>> mtls) {
    _materials = std::move(mtls);
  }

  void set_feature_manager(feature_manager &&feature) {
    _feature_manager = std::move(feature);
  }

  feature_manager &features() { return _feature_manager; }

private:
  struct file_patch {
    vdb::math::transform::ptr _resolution;
    vertex_grid::ptr _vertex;
    pixel_grid::ptr _pixel;
  };

public:
  future<> write(random_access_writer &writer);

  future<> read(const file_path &file);

  future<> read(io_virtual_file file);

  static voxel_pot create_voxel_pot() { return voxel_pot(); }

  enum class composite_type { op_union, op_intersection, op_difference };

  void composite(voxel_pot &pot, const composite_type &type);

  bool ray_test(const flywave::extray3<double> &ray, vdb::vec3d &p);

  void clean() { return clear_unuse_materials(); }

private:
  voxel_pot(vertex_grid::ptr vertex, pixel_grid::ptr pixel,
            vdb::math::transform::ptr);

  void merge_materials(voxel_pot &tpot);

  void clear_unuse_materials();

  future<> read(random_access_reader &reader);

private:
  io_virtual_file _file;
  vdb::math::transform::ptr _resolution;
  vertex_grid::ptr _vertex;
  pixel_grid::ptr _pixel;
  std::vector<shared_ptr<material_data>> _materials;
  feature_manager _feature_manager;
};

} // namespace voxelize
} // namespace flywave
