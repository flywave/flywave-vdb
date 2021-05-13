#pragma once

#include "mesh_adapter.hh"
#include "trees.hh"

#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>

namespace flywave {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

class voxel_pixel_sampler;

class voxel_pixel {
  friend class voxel_pixel_sampler;

public:
  voxel_pixel()
      : _resolution(vdb::math::Transform::createLinearTransform()),
        _vertex(vertex_grid::create(std::numeric_limits<float>::max())),
        _pixel(pixel_grid::create()) {
    _vertex->setTransform(_resolution);
    _vertex->setGridClass(openvdb::GRID_LEVEL_SET);
  }

  voxel_pixel(vdb::math::Transform::Ptr transfrom)
      : _resolution(transfrom),
        _vertex(vertex_grid::create(std::numeric_limits<float>::max())),
        _pixel(pixel_grid::create()) {
    _vertex->setTransform(_resolution);
    _vertex->setGridClass(openvdb::GRID_LEVEL_SET);
  }
  voxel_pixel(vertex_grid::Ptr vertex, pixel_grid::Ptr pixel,
              vdb::math::Transform::Ptr);
  voxel_pixel(voxel_pixel *grid);

  vdb::math::Transform::Ptr voxel_resolution() const { return _resolution; }

  vertex_grid::Ptr get_voxel_grid() const { return _vertex; }

  pixel_grid::Ptr get_pixel_grid() const { return _pixel; }

  bool is_sdf() const;

  double get_volume() const;

  double get_area() const;

  int64_t get_memory_size() const;

  void set_voxel_grid(vertex_grid::Ptr ptr) {
    _vertex = ptr;
    _vertex->setTransform(_resolution);
  }

  void set_pixel_grid(pixel_grid::Ptr ptr) { _pixel = ptr; }

  bool is_empty() { return _vertex == nullptr; }

  std::vector<std::shared_ptr<material_data>> get_materials();

  void set_materials(std::vector<std::shared_ptr<material_data>> mtls);

  void add_material(std::shared_ptr<material_data> mtl);

  void remove_material(material_id_t id);

  std::shared_ptr<material_data> get_material(material_id_t id);

  bool has_material(material_id_t id) const;

  size_t materials_count() const;

  void clear_materials() const;

  std::vector<std::shared_ptr<feature_data>> get_features();

  void set_features(std::vector<std::shared_ptr<feature_data>> feats);

  void add_features(std::shared_ptr<feature_data> feat);

  std::shared_ptr<feature_data> get_feature(local_feature_id_t id);

  void remove_feature(local_feature_id_t id);

  bool has_feature(local_feature_id_t id) const;

  size_t features_count() const;

  void clear_features() const;

public:
  bool write(const std::string &file);

  bool read(const std::string &file);

  static std::shared_ptr<voxel_pixel> create_voxel_pixel() {
    return std::make_shared<voxel_pixel>();
  }

  enum class composite_type : uint32_t {
    op_union,
    op_intersection,
    op_difference
  };

  void composite(voxel_pixel &pot, const composite_type &type);

  bool ray_test(const vdb::math::Ray<double> &ray, openvdb::Vec3d &p);

  bool ray_test(const std::vector<vdb::math::Ray<double>> &rays,
                std::vector<openvdb::Vec3d> &ps);

  void clear();

  pixel_grid::Ptr extract_color(voxel_pixel &spot);

  void fill_color(voxel_pixel &spot, pixel_grid::Ptr _colors);

  vdb::BBoxd eval_max_min_elevation(vdb::BBoxd _in);

  void clear_unuse_materials();

  void clear_unuse_features();

  openvdb::Vec3d get_gradient(const openvdb::Vec3d &pos) const;

  bool eval_gradients(openvdb::Vec3d *gradients, int stride,
                      const openvdb::Vec3d *pos, int num_pos,
                      bool normalize) const;

  double calc_positive_density() const;

private:
  void merge_materials(voxel_pixel &tpot);

private:
  vdb::math::Transform::Ptr _resolution;
  vertex_grid::Ptr _vertex;
  pixel_grid::Ptr _pixel;
  material_meta_map::Ptr _materials;
  feature_meta_map::Ptr _features;
};

} // namespace flywave
