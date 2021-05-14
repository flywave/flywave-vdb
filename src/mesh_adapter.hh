#pragma once

#include <fstream>
#include <iostream>

#include "color_extract.hh"
#include "feature_data.hh"
#include "material_data.hh"
#include "types.hh"
#include "uv_policy.hh"

#include <openvdb/Types.h>

namespace flywave {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

struct texture_sampler {
  std::shared_ptr<uv_policy> _policy;
  std::shared_ptr<color_extract> _extract;
};

class material_group {
public:
  material_group(std::shared_ptr<material_data> mdata,
                 const texture_sampler &sampler)
      : _texture_sampler(std::make_shared<texture_sampler>(sampler)),
        _material_data(mdata) {}

  material_group(std::shared_ptr<material_data> mdata)
      : _material_data(mdata) {}

  material_group() = default;

  material_id_t material_id() const { return _material_data->_material_id; }

  const material_data &material() const { return *_material_data; }

  std::shared_ptr<material_data> material_ptr() const { return _material_data; }

  texture_sampler &sampler() const { return *_texture_sampler; }

  bool has_texture_sampler() const { return _texture_sampler != nullptr; }

private:
  std::shared_ptr<texture_sampler> _texture_sampler;
  std::shared_ptr<material_data> _material_data;
};

struct data_triangle {
  triangle3<double> _triangle;
  uint16_t _material_id;

  const openvdb::Vec3d &operator[](size_t index) { return _triangle[index]; }
};

class mesh_adapter;
class vertext_sampler;
class voxel_pixel_sampler;

class triangles_stream {
  friend class mesh_adapter;
  friend class vertext_sampler;
  friend class voxel_pixel_sampler;

public:
  virtual ~triangles_stream() = default;

  triangle3<double> find_triangle_transfromed(uint32_t face_index) {
    data_triangle tri = find_triangle(face_index);
    return triangle3<double>(_xform->worldToIndex(tri._triangle[0]),
                             _xform->worldToIndex(tri._triangle[1]),
                             _xform->worldToIndex(tri._triangle[2]));
  }

  virtual size_t polygonCount() const = 0;

  virtual size_t pointCount() const = 0;

  virtual size_t vertexCount(size_t n) const { return 3; }

  virtual void getIndexSpacePoint(size_t n, size_t v,
                                  openvdb::Vec3d &pos) const = 0;

  virtual void set_transfrom(vdb::math::Transform::Ptr xfrom) {
    _xform = xfrom;
  }

  virtual void set_matrix(openvdb::Mat4d matrix) { _matrix44 = matrix; }

  openvdb::BBoxd compute_boundbox() {
    openvdb::BBoxd box;
    auto pc = polygonCount();

    for (auto i = 0; i < pc; i++) {
      auto tri = find_triangle_transfromed(i);
      box.expand(tri[0]);
      box.expand(tri[1]);
      box.expand(tri[2]);
    }
    return box;
  }

  virtual data_triangle find_triangle(uint32_t face_index) = 0;

  openvdb::Mat4d world_to_local() { return _matrix44.inverse(); }

private:
  vdb::math::Transform::Ptr _xform;

protected:
  openvdb::Mat4d _matrix44;
};

class mesh_adapter {
private:
  void init_materials();

public:
  mesh_adapter(std::unique_ptr<triangles_stream> stream)
      : _stream(std::move(stream)) {
    init_materials();
  }

  void add_material(const material_group &group) {
    _materials.emplace(group.material_id(), group);
  }

  const material_group &find_material(material_id_t index) {
    return _materials[index];
  }

  bool has_materials() const { return _materials.size() > 0; }

  std::unique_ptr<triangles_stream> _stream;
  std::map<material_id_t, material_group> _materials;
};

class material_merge_transfrom {
public:
  material_merge_transfrom(std::vector<std::shared_ptr<material_data>> mtls)
      : _materials(std::move(mtls)) {}

  void merge(std::vector<std::shared_ptr<material_data>> materials) {
    for (auto pt : materials) {
      auto qpt = find_same_material(_materials, pt);
      if (!qpt) {
        _materials.emplace_back(pt);
      }
      material_id_t old = pt->_material_id;
      if (!qpt)
        pt->_material_id = _materials.size() - 1;
      else
        pt->_material_id = qpt->_material_id;

      _mapping.emplace(old, pt->_material_id);
    }
  }

  material_id_t new_material_id(material_id_t id) const {
    return _mapping.find(id)->second;
  }

  std::vector<std::shared_ptr<material_data>> materials() {
    return std::move(_materials);
  }

private:
  std::shared_ptr<material_data>
  find_same_material(std::vector<std::shared_ptr<material_data>> &mates,
                     std::shared_ptr<material_data> mat) {
    for (auto pt : mates) {
      if (*mat == *pt)
        return pt;
    }

    return nullptr;
  }

protected:
  std::vector<std::shared_ptr<material_data>> _materials;
  std::map<material_id_t, material_id_t> _mapping;
};

} // namespace flywave
