#pragma once

#include "color_extract.hh"
#include "st_policy.hh"
#include "types.hh"

#include <openvdb/Types.h>

namespace flywave {
namespace voxelize {

struct texture_sampler {
  std::shared_ptr<st_policy> _policy;
  std::shared_ptr<color_extract> _extract;
};

struct material_data {
  material_id_t _material_id = -1;

  enum { BASE = 0, LAMBERT = 1, PHONG = 2, PBR = 3 };
  enum { COLOR = 0x1, TEXTURE = 0x2, BUMP = 0x4 };

  uint32_t type = 0;
  uint16_t mode{0};

  Eigen::Matrix<uint8_t, 3, 1> color{255, 255, 255};

  Eigen::Matrix<uint8_t, 3, 1> ambient{255, 255, 255};
  Eigen::Matrix<uint8_t, 3, 1> emissive{0, 0, 0};

  Eigen::Matrix<uint8_t, 3, 1> specular{0, 0, 0};

  float opacity{1.0};

  float shininess{20.0f};

  float metallic{0.0f};
  float roughness{1.0f};
  float reflectance{0.5f};

  float clearcoat_thickness{0.0f};
  float clearcoat_roughness{0.0f};

  float anisotropy{0.f};
  float anisotropy_rotation{0.0f};

  material_data(const material_data &) = default;
  material_data() = default;
  material_data(material_data &&) = default;

  bool operator==(const material_data &p) const {
    if (type == p.type) {
      switch (type) {
      case BASE:
        return mode == p.mode && color == p.color &&
              openvdb::math::isApproxEqual(opacity, p.opacity);
      case LAMBERT:
        return mode == p.mode && color == p.color &&
              openvdb::math::isApproxEqual(opacity, p.opacity) &&
               ambient == p.ambient && emissive == p.emissive;
      case PHONG:
        return mode == p.mode && color == p.color &&
              openvdb::math::isApproxEqual(opacity, p.opacity) &&
               ambient == p.ambient && emissive == p.emissive &&
               specular == p.specular &&
              openvdb::math::isApproxEqual(shininess, p.shininess);
      case PBR:
        return mode == p.mode && color == p.color &&
              openvdb::math::isApproxEqual(opacity, p.opacity) &&
              openvdb::math::isApproxEqual(metallic, p.metallic) &&
              openvdb::math::isApproxEqual(roughness, p.roughness) &&
              openvdb::math::isApproxEqual(reflectance, p.reflectance) &&
              openvdb::math::isApproxEqual(clearcoat_thickness,
                                              p.clearcoat_thickness) &&
              openvdb::math::isApproxEqual(clearcoat_roughness,
                                              p.clearcoat_roughness) &&
              openvdb::math::isApproxEqual(anisotropy, p.anisotropy) &&
              openvdb::math::isApproxEqual(anisotropy_rotation,
                                              p.anisotropy_rotation);
      default:
        break;
      }
    }
    return false;
  }
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
  triangle3<float> _triangle;
  uint16_t _material_id;
    
  const openvdb::Vec3f &operator[](size_t index) {
    return _triangle[index];
  }
};

class mesh_adapter;
class vertext_sampler;
class micronizer;

class triangles_stream {
  friend class mesh_adapter;
  friend class vertext_sampler;
  friend class micronizer;

public:
  triangle3<float> find_triangle_transfromed(uint32_t face_index) {
    data_triangle tri = find_triangle(face_index);
    return triangle3<float>(_xform->worldToIndex(tri._triangle[0]),
                            _xform->worldToIndex(tri._triangle[1]),
                            _xform->worldToIndex(tri._triangle[2]));
  }

  virtual size_t polygon_count() const = 0;

  virtual void set_transfrom(openvdb::math::Transform::Ptr xfrom) {
    _xform = xfrom;
  }

  virtual void set_matrix(openvdb::Mat4d matrix) {
    _matrix44 = matrix;
  }

  openvdb::BBoxd compute_boundbox() {
    openvdb::BBoxd box;
    auto pc = polygon_count();

    for (auto i = 0; i < pc; i++) {
      auto tri = find_triangle_transfromed(i);
      box.expand(tri[0]);
      box.expand(tri[1]);
      box.expand(tri[2]);
    }
    return box;
  }

  virtual ~triangles_stream() = default;

  virtual data_triangle find_triangle(uint32_t face_index) = 0;

private:
  openvdb::math::Transform::Ptr _xform;

protected:
  openvdb::Mat4d _matrix44;
};

class mesh_adapter {
public:
  mesh_adapter(std::unique_ptr<triangles_stream> stream)
      : _stream(std::move(stream)) {}

  void add_material(const material_group &group) {
    auto gp = group;
    //    gp.material_ptr()->_material_id = _materials.size();
    _materials.emplace(gp.material_id(), gp);
  }

  const material_group &find_material(material_id_t index) {
    return _materials[index];
  }

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
} // namespace voxelize
} // namespace flywave
