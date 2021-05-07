#pragma once

#include <openvdb/openvdb.h>

#include <unordered_map>
#include <vector>

namespace flywave {

const int none_mtl_id = -1;

class mesh_data {
public:
  mesh_data();
  ~mesh_data();

  mesh_data duplicate();

  bool is_valid();

  std::vector<openvdb::Vec3s> &vertices();
  std::vector<openvdb::Vec2s> &texcoords();
  std::vector<openvdb::Vec3s> &normals();

  std::unordered_map<int, std::vector<std::vector<openvdb::Vec3I>>> &
  smooth_faces_map() {
    return _smooth_faces_map;
  }

  std::unordered_map<int, std::vector<openvdb::Vec3I>> &mtl_faces_map() {
    return _mtl_faces_map;
  }

  std::unordered_map<int, std::vector<openvdb::Vec3I>> &mtl_normals_map() {
    return _mtl_normal_map;
  }

  std::unordered_map<int, std::vector<openvdb::Vec3I>> &mtl_texcoords_map() {
    return _mtl_texcoord_map;
  }

  void add_vertice(openvdb::Vec3s v);
  void add_vertice(std::vector<openvdb::Vec3s> v);

  void add_texcoord(openvdb::Vec2s t);
  void add_texcoord(std::vector<openvdb::Vec2s> t);

  void add_normal(openvdb::Vec3s n);
  void add_normal(std::vector<openvdb::Vec3s> n);

  bool has_normal() { return _normals.size(); }

  bool has_texcoord() { return _texcoords.size(); }

  bool has_mtl_normal() { return _mtl_normal_map.size(); }

  bool has_mtl_texcoord() { return _mtl_texcoord_map.size(); }

  bool has_smmoth_face() { return _smooth_faces_map.size(); }

  void add_smooth_face(int mtl_id, const std::vector<openvdb::Vec3I> &fs);

  void add_smooth_faces(int mtl_id,
                        const std::vector<std::vector<openvdb::Vec3I>> &fs);

  void add_mtl_face(int mtl_id, const openvdb::Vec3I &f);

  void add_mtl_faces(int mtl_id, const std::vector<openvdb::Vec3I> &f);

  void add_mtl_normal(int mtl_id, const openvdb::Vec3I &f);

  void add_mtl_normals(int mtl_id, const std::vector<openvdb::Vec3I> &f);

  void add_mtl_texcoord(int mtl_id, const openvdb::Vec3I &f);

  void add_mtl_texcoords(int mtl_id, const std::vector<openvdb::Vec3I> &f);

  void clear();

private:
  std::vector<openvdb::Vec3s> _vertices;
  std::vector<openvdb::Vec2s> _texcoords;
  std::vector<openvdb::Vec3s> _normals;

  std::unordered_map<int, std::vector<openvdb::Vec3I>> _mtl_faces_map;
  std::unordered_map<int, std::vector<openvdb::Vec3I>> _mtl_normal_map;
  std::unordered_map<int, std::vector<openvdb::Vec3I>> _mtl_texcoord_map;
  std::unordered_map<int, std::vector<std::vector<openvdb::Vec3I>>>
      _smooth_faces_map;
};
} // namespace flywave
