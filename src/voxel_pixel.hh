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

  vdb::math::Transform::Ptr voxel_resolution() const { return _resolution; }

  vertex_grid::Ptr get_voxel_grid() const { return _vertex; }

  pixel_grid::Ptr get_pixel_grid() const { return _pixel; }

  void set_voxel_grid(vertex_grid::Ptr ptr) {
    _vertex = ptr;
    _vertex->setTransform(_resolution);
  }

  void set_pixel_grid(pixel_grid::Ptr ptr) { _pixel = ptr; }

  bool is_empty() { return _vertex == nullptr; }

  std::vector<std::shared_ptr<material_data>> materials() {
    return std::move(_materials);
  }

  void set_materials(std::vector<std::shared_ptr<material_data>> mtls) {
    _materials = std::move(mtls);
  }

public:
  void write(const std::string &file);

  void read(const std::string &file);

  static voxel_pixel create_voxel_pixel() { return voxel_pixel(); }

  enum class composite_type : uint32_t {
    op_union,
    op_intersection,
    op_difference
  };

  void composite(voxel_pixel &pot, const composite_type &type);

  bool ray_test(const vdb::math::Ray<double> &ray, openvdb::Vec3d &p);

  void clear() { return clear_unuse_materials(); }

private:
  voxel_pixel(vertex_grid::Ptr vertex, pixel_grid::Ptr pixel,
              vdb::math::Transform::Ptr);

  void merge_materials(voxel_pixel &tpot);

  void clear_unuse_materials();

private:
  vdb::math::Transform::Ptr _resolution;
  vertex_grid::Ptr _vertex;
  pixel_grid::Ptr _pixel;
  std::vector<std::shared_ptr<material_data>> _materials;
};

} // namespace flywave
