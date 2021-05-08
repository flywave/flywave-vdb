#pragma once

#include <string>
#include <vector>

#include "mesh_data.hh"
#include "texture_data.hh"
#include "voxel_mesh.hh"

namespace flywave {

class voxel_mesh_builder {
protected:
  std::string _name;
  std::vector<mesh_data> _datas;
  std::unordered_map<int, std::shared_ptr<material>> _mtls;
  std::unordered_map<std::string, std::shared_ptr<texture>> _textures;

public:
  voxel_mesh_builder() : _name(""), _mtls() {}

  virtual ~voxel_mesh_builder() = default;

  std::shared_ptr<voxel_mesh> build_mesh();

  std::vector<triangle> build_triangles();

  void set_name(const std::string &nm) { _name = nm; }

  const std::string &get_name() const {
    return _name;
  }

  std::vector<mesh_data> &get_mesh_datas() { return _datas; }

  void add_mesh_data(mesh_data &&dt) { _datas.emplace_back(std::move(dt)); }

  int add_material_data(material_data &&md, int index = -1);

  int add_material(std::shared_ptr<material> mt, int index = -1);

  std::unordered_map<int, std::shared_ptr<material>> &get_materials() {
    return _mtls;
  }

  void add_texture(const std::string &name, texture_data &&buf) {
    _textures.emplace(name, std::make_shared<texture>());
    auto &_tex = _textures[name];
    _tex->name = name;

    std::shared_ptr<texture2d<vdb::math::Vec4<uint8_t>>> dest =
        texture2d<vdb::math::Vec4<uint8_t>>::create(
            std::make_pair(buf.width, buf.height));
    if (buf.format == pixel_format::RGB) {
      auto t = std::make_shared<texture2d<vdb::math::Vec3<uint8_t>>>(
          std::make_pair(buf.width, buf.height),
          (const uint8_t *)buf.data.data(), buf.data.size());
      transform_color(*t, *dest, [](vdb::math::Vec3<uint8_t> c) {
        return vdb::math::Vec4<uint8_t>{static_cast<uint8_t>(c.x()),
                                        static_cast<uint8_t>(c.y()),
                                        static_cast<uint8_t>(c.z()), 255};
      });
    } else {
      dest = std::make_shared<texture2d<vdb::math::Vec4<uint8_t>>>(
          std::make_pair(buf.width, buf.height),
          (const uint8_t *)buf.data.data(), buf.data.size());
    }
    _tex->data = dest;
  }

  void add_texture(const std::string &name,
                   std::shared_ptr<texture2d<vdb::math::Vec4<uint8_t>>> t) {
    _textures.emplace(name, std::make_shared<texture>());
    auto &_tex = _textures[name];
    _tex->name = name;
    _tex->data = t;
  }

  std::unordered_map<std::string, std::shared_ptr<texture>> &get_textures() {
    return _textures;
  }

  bool texture_exist(const std::string &name) {
    return _textures.find(name) != _textures.end();
  }

  void close() { ; }

  bool has_texture() { return _textures.size(); }
};

} // namespace flywave
