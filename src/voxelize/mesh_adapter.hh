#pragma once

#include <flywave/core/io_file_pickle.hh>
#include <flywave/core/range.hh>
#include <flywave/math/triangle.hh>
#include <flywave/math/vector_lib.hh>
#include <flywave/voxelize/color_extract.hh>
#include <flywave/voxelize/st_policy.hh>
#include <flywave/voxelize/types.hh>

namespace flywave {
namespace voxelize {

struct texture_sampler {
  shared_ptr<st_policy> _policy;
  shared_ptr<color_extract> _extract;
};

struct material_data {
  material_id_t _material_id = -1;

  enum { BASE = 0, LAMBERT = 1, PHONG = 2, PBR = 3 };
  enum { COLOR = 0x1, TEXTURE = 0x2, BUMP = 0x4 };
  uint32_t type = 0;
  uint16_t mode{0};

  color3i color{255, 255, 255};

  color3i ambient{255, 255, 255};
  color3i emissive{0, 0, 0};

  color3i specular{0, 0, 0};

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
               flywave::math::is_approx_equal(opacity, p.opacity);
      case LAMBERT:
        return mode == p.mode && color == p.color &&
               flywave::math::is_approx_equal(opacity, p.opacity) &&
               ambient == p.ambient && emissive == p.emissive;
      case PHONG:
        return mode == p.mode && color == p.color &&
               flywave::math::is_approx_equal(opacity, p.opacity) &&
               ambient == p.ambient && emissive == p.emissive &&
               specular == p.specular &&
               flywave::math::is_approx_equal(shininess, p.shininess);
      case PBR:
        return mode == p.mode && color == p.color &&
               flywave::math::is_approx_equal(opacity, p.opacity) &&
               flywave::math::is_approx_equal(metallic, p.metallic) &&
               flywave::math::is_approx_equal(roughness, p.roughness) &&
               flywave::math::is_approx_equal(reflectance, p.reflectance) &&
               flywave::math::is_approx_equal(clearcoat_thickness,
                                              p.clearcoat_thickness) &&
               flywave::math::is_approx_equal(clearcoat_roughness,
                                              p.clearcoat_roughness) &&
               flywave::math::is_approx_equal(anisotropy, p.anisotropy) &&
               flywave::math::is_approx_equal(anisotropy_rotation,
                                              p.anisotropy_rotation);
      default:
        break;
      }
    }
    return false;
  }

  template <typename Describer> auto describe_type(Describer f) {
    return f(_material_id, type, mode, color, ambient, emissive, specular,
             opacity, shininess, metallic, roughness, reflectance,
             clearcoat_thickness, clearcoat_roughness, anisotropy,
             anisotropy_rotation);
  }
};

class material_group {
public:
  material_group(shared_ptr<material_data> mdata,
                 const texture_sampler &sampler)
      : _texture_sampler(make_shared<texture_sampler>(sampler)),
        _material_data(mdata) {}

  material_group(shared_ptr<material_data> mdata) : _material_data(mdata) {}

  material_group() = default;

  material_id_t material_id() const { return _material_data->_material_id; }

  const material_data &material() const { return *_material_data; }

  shared_ptr<material_data> material_ptr() const { return _material_data; }

  texture_sampler &sampler() const { return *_texture_sampler; }

  bool has_texture_sampler() const { return _texture_sampler != nullptr; }

private:
  shared_ptr<texture_sampler> _texture_sampler;
  shared_ptr<material_data> _material_data;
};

struct data_triangle {
  triangle3<float> _triangle;
  uint16_t _material_id;

  const vector3<float> &operator[](size_t index) { return _triangle[index]; }
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
    return triangle3<float>(_xform->world_to_index(tri._triangle[0]),
                            _xform->world_to_index(tri._triangle[1]),
                            _xform->world_to_index(tri._triangle[2]));
  }

  virtual size_t polygon_count() const = 0;

  virtual void set_transfrom(vdb::math::transform::ptr xfrom) {
    _xform = xfrom;
  }

  virtual void set_matrix(matrix44<double> matrix) { _matrix44 = matrix; }

  bbox3d compute_boundbox() {
    bbox3d box;
    auto pc = polygon_count();

    for (auto i = 0; i < pc; i++) {
      auto tri = find_triangle_transfromed(i);
      box.extend(tri[0]);
      box.extend(tri[1]);
      box.extend(tri[2]);
    }
    return box;
  }

  virtual ~triangles_stream() = default;

  virtual data_triangle find_triangle(uint32_t face_index) = 0;

private:
  vdb::math::transform::ptr _xform;

protected:
  matrix44<double> _matrix44;
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
  material_merge_transfrom(std::vector<shared_ptr<material_data>> mtls)
      : _materials(std::move(mtls)) {}

  void merge(std::vector<shared_ptr<material_data>> materials) {
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

  std::vector<shared_ptr<material_data>> materials() {
    return std::move(_materials);
  }

private:
  shared_ptr<material_data>
  find_same_material(std::vector<shared_ptr<material_data>> &mates,
                     shared_ptr<material_data> mat) {
    for (auto pt : mates) {
      if (*mat == *pt)
        return pt;
    }

    return nullptr;
  }

protected:
  std::vector<shared_ptr<material_data>> _materials;
  std::map<material_id_t, material_id_t> _mapping;
};
} // namespace voxelize
} // namespace flywave
