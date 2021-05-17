#pragma once

#include "mesh_adapter.hh"
#include "texture2d.hh"

#include <openvdb/openvdb.h>

#include <cassert>
#include <unordered_map>
#include <vector>

namespace flywave {

struct triangle {
  openvdb::Vec3d ver[3];
  openvdb::Vec2d texcoord[3];
  int mtl_id{-1};
};

class voxel_mesh {
protected:
  std::vector<triangle> _triangles;

public:
  const triangle &find_triangle(uint32_t index) const {
    assert(index < _triangles.size());
    return _triangles[index];
  }

  void clear() { _triangles.clear(); }

  void add_triangle(triangle tr) { _triangles.emplace_back(std::move(tr)); }

  void add_tris(std::vector<triangle> &&tr) { _triangles = std::move(tr); }

  size_t size() const { return _triangles.size(); }

  vdb::BBoxd get_bounds() const; 
};

struct texture {
  std::string name;
  std::shared_ptr<texture2d<vdb::math::Vec4<uint8_t>>> data;

  texture() : name("") {}
  texture(texture &&) = default;

  vdb::math::Vec4<uint8_t> get(size_t x, size_t y) { return data->color(x, y); }

  explicit operator bool() const { return data->bytes(); }
};

class material {
public:
  uint16_t type = 0;
  uint16_t mode = 1;

  std::string ambient_texname;
  std::string diffuse_texname;
  std::string specular_texname;
  std::string specular_highlight_texname;
  std::string bump_texname;
  std::string displacement_texname;
  std::string alpha_texname;
  std::string reflection_texname;

  vdb::math::Vec3<float> color;
  vdb::math::Vec3<float> ambient;
  vdb::math::Vec3<float> emissive;
  vdb::math::Vec3<float> specular;

  int illum{2};
  float shiness{0.0f};
  float transparency{0.0f};

  float metallic{0.0f};
  float roughness{1.0f};
  float reflectance{0.5f};

  float clearcoat_thickness{0.0f};
  float clearcoat_roughness{0.0f};

  float anisotropy{0.f};
  float anisotropy_rotation{0.0f};

  std::string name;

  material()
      : color(0.8, 0.8, 0.8), ambient(1, 1, 1), emissive(0, 0, 0),
        specular(0, 0, 0) {}
  material(const material_data &data);
  material(material &&) = default;
  material(const material &) = default;

  material &operator=(const material &) = default;

  void set_reflection_texname(const std::string &name) {
    if (name.empty())
      return;
    reflection_texname = name;
    mode = 2;
  }

  void set_diffuse_texname(const std::string &name) {
    if (name.empty())
      return;
    diffuse_texname = name;
    mode = 2;
  }

  void set_bump_texname(const std::string &name) {
    if (name.empty())
      return;
    bump_texname = name;
    mode = 3;
  }

  void set_specular_texname(const std::string &name) {
    if (name.empty())
      return;
    specular_texname = name;
    mode = 2;
  }

  void set_transparency(float tran) { transparency = tran; }

  float get_transparency() const { return transparency; }

  void set_shiness(float s) { shiness = s; }

  float get_shiness() const { return shiness; }

  const vdb::math::Vec3<float> &get_color() const { return color; }

  void set_color(const vdb::math::Vec3<float> &cl) { color = cl; }

  const vdb::math::Vec3<float> &get_specular() const { return specular; }

  void set_specular(const vdb::math::Vec3<float> &cl) { specular = cl; }

  const vdb::math::Vec3<float> &get_ambient() const { return ambient; }

  void set_ambient(const vdb::math::Vec3<float> &cl) { ambient = cl; }

  const vdb::math::Vec3<float> &get_emissive() const { return emissive; }

  void set_emissive(const vdb::math::Vec3<float> &cl) { emissive = cl; }

  const std::string &get_name() const { return name; }

  void set_name(const std::string &str) { name = str; }

  float get_metallic() { return metallic; };
  float get_roughness() { return roughness; };
  float get_reflectance() { return reflectance; };
  float get_clearcoat_thickness() { return clearcoat_thickness; };
  float get_clearcoat_roughness() { return clearcoat_roughness; };
  float get_anisotropy() { return anisotropy; };
  float get_anisotropy_rotation() { return anisotropy_rotation; };

  void set_metallic(float f) { metallic = f; };
  void set_roughness(float f) { roughness = f; };
  void set_reflectance(float f) { reflectance = f; };
  void set_clearcoat_thickness(float f) { clearcoat_thickness = f; };
  void set_clearcoat_roughness(float f) { clearcoat_roughness = f; };
  void set_anisotropy(float f) { anisotropy = f; };
  void set_anisotropy_rotation(float f) { anisotropy_rotation = f; };

  material &operator=(const material_data &data);

  explicit operator material_data() const;
};

class mesh_adapter;

class voxel_mesh_adapter : public triangles_stream {
private:
  std::unordered_map<int, std::shared_ptr<material_data>> _mt_map;
  std::unordered_map<int, std::shared_ptr<material>> _map;
  std::unordered_map<std::string, std::shared_ptr<texture>> _tex_map;
  mutable std::shared_ptr<voxel_mesh> _mesh;

public:
  voxel_mesh_adapter(
      std::shared_ptr<voxel_mesh> hm,
      std::unordered_map<int, std::shared_ptr<material>> &map,
      std::unordered_map<std::string, std::shared_ptr<texture>> &tex_map);

  size_t polygonCount() const override { return _mesh->size(); }

  size_t pointCount() const override { return _mesh->size() * 3; }

  size_t vertexCount(size_t n) const override { return 3; }

  void getIndexSpacePoint(size_t n, size_t v,
                          openvdb::Vec3d &pos) const override {
    auto tri = _mesh->find_triangle(n);
    pos = _matrix44.transformH(tri.ver[v]);
  }

  data_triangle find_triangle(uint32_t face_index) override {
    auto tri = _mesh->find_triangle(face_index);
    openvdb::Vec3d v1 = _matrix44.transformH(tri.ver[0]);
    openvdb::Vec3d v2 = _matrix44.transformH(tri.ver[1]);
    openvdb::Vec3d v3 = _matrix44.transformH(tri.ver[2]);

    data_triangle mdt{triangle3<double>{v1, v2, v3},
                      static_cast<uint16_t>(tri.mtl_id)};
    return mdt;
  }

  friend class mesh_adapter;
};

class color_extract_impl : public color_extract {
  texture &_tx;

public:
  color_extract_impl(texture &tx) : _tx(tx) {}

  virtual color_type extract(const material_group &fgroup, const vdb::Vec2d &uv,
                             const triangle3<double> &tri);
};

class uv_reader_impl : public uv_coord_reader {
public:
  uv_reader_impl(const voxel_mesh &hm) : _hm(hm) {}

  triangle2<double> get(uint32_t index) override;

private:
  const voxel_mesh &_hm;
};
} // namespace flywave
