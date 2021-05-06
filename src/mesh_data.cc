#include "mesh_data.hh"

namespace flywave {

mesh_data::mesh_data() {
  _vertices.clear();
  _faces.clear();
}

mesh_data mesh_data::duplicate() {
  mesh_data mesh;
  mesh.add_vertice(_vertices);
  mesh.add_face(_faces);
  mesh.add_texcoord(_texcoords);
  mesh.add_normal(_normals);

  for (auto &pr : _mtl_faces_map) {
    mesh.add_mtl_faces(pr.first, pr.second);
  }
  for (auto &pr : _mtl_normal_map) {
    mesh.add_mtl_normals(pr.first, pr.second);
  }
  for (auto &pr : _mtl_texcoord_map) {
    mesh.add_mtl_texcoords(pr.first, pr.second);
  }
  for (auto &pr : _smooth_faces_map) {
    mesh.add_smooth_faces(pr.first, pr.second);
  }

  return mesh;
}

mesh_data::~mesh_data() {}

bool mesh_data::is_valid() {
  if (_faces.size() > 0 && _vertices.size() > 0) {
    return true;
  }
  return false;
}

std::vector<openvdb::Vec3s> &mesh_data::vertices() { return _vertices; }

std::vector<openvdb::Vec4I> &mesh_data::faces() { return _faces; }
std::vector<openvdb::Vec2s> &mesh_data::texcoords() { return _texcoords; }
std::vector<openvdb::Vec3s> &mesh_data::normals() { return _normals; }

void mesh_data::add_vertice(openvdb::Vec3s v) { _vertices.push_back(v); }

void mesh_data::add_vertice(std::vector<openvdb::Vec3s> v) {
  _vertices.insert(_vertices.end(), v.begin(), v.end());
}

void mesh_data::add_face(openvdb::Vec4I f) { _faces.push_back(f); }

void mesh_data::add_face(std::vector<openvdb::Vec4I> f) {
  _faces.insert(_faces.end(), f.begin(), f.end());
}

void mesh_data::add_texcoord(openvdb::Vec2s t) { _texcoords.push_back(t); }

void mesh_data::add_texcoord(std::vector<openvdb::Vec2s> t) {
  _texcoords.insert(_texcoords.end(), t.begin(), t.end());
}

void mesh_data::add_normal(openvdb::Vec3s n) { _normals.push_back(n); }

void mesh_data::add_normal(std::vector<openvdb::Vec3s> n) {
  _normals.insert(_normals.end(), n.begin(), n.end());
}

void mesh_data::add_smooth_face(int mtl_id,
                                const std::vector<openvdb::Vec3I> &fs) {
  auto it = _smooth_faces_map.find(mtl_id);
  if (it == _smooth_faces_map.end())
    _smooth_faces_map.emplace(mtl_id,
                              std::vector<std::vector<openvdb::Vec3I>>{});
  it = _smooth_faces_map.find(mtl_id);
  auto &g = it->second;
  g.emplace_back(fs);
}

void mesh_data::add_smooth_faces(
    int mtl_id, const std::vector<std::vector<openvdb::Vec3I>> &fs) {
  auto it = _smooth_faces_map.find(mtl_id);
  if (it == _smooth_faces_map.end())
    _smooth_faces_map.emplace(mtl_id,
                              std::vector<std::vector<openvdb::Vec3I>>{});

  auto &g = _smooth_faces_map.find(mtl_id)->second;
  g.insert(g.end(), fs.begin(), fs.end());
}

void mesh_data::add_mtl_face(int mtl_id, const openvdb::Vec3I &f) {
  auto it = _mtl_faces_map.find(mtl_id);
  if (it == _mtl_faces_map.end())
    _mtl_faces_map.emplace(mtl_id, std::vector<openvdb::Vec3I>{});

  it = _mtl_faces_map.find(mtl_id);
  auto &g = it->second;
  g.emplace_back(f);
}

void mesh_data::add_mtl_faces(int mtl_id,
                              const std::vector<openvdb::Vec3I> &f) {
  auto it = _mtl_faces_map.find(mtl_id);
  if (it == _mtl_faces_map.end())
    _mtl_faces_map.emplace(mtl_id, std::vector<openvdb::Vec3I>{});

  auto &g = _mtl_faces_map.find(mtl_id)->second;
  g.insert(g.end(), f.begin(), f.end());
}

void mesh_data::add_mtl_normal(int mtl_id, const openvdb::Vec3I &f) {
  auto it = _mtl_normal_map.find(mtl_id);
  if (it == _mtl_normal_map.end())
    _mtl_normal_map.emplace(mtl_id, std::vector<openvdb::Vec3I>{});

  it = _mtl_normal_map.find(mtl_id);
  auto &g = it->second;
  g.emplace_back(f);
}

void mesh_data::add_mtl_normals(int mtl_id,
                                const std::vector<openvdb::Vec3I> &f) {
  auto it = _mtl_normal_map.find(mtl_id);
  if (it == _mtl_normal_map.end())
    _mtl_normal_map.emplace(mtl_id, std::vector<openvdb::Vec3I>{});

  auto &g = _mtl_normal_map.find(mtl_id)->second;
  g.insert(g.end(), f.begin(), f.end());
}

void mesh_data::add_mtl_texcoord(int mtl_id, const openvdb::Vec3I &f) {
  auto it = _mtl_texcoord_map.find(mtl_id);
  if (it == _mtl_texcoord_map.end())
    _mtl_texcoord_map.emplace(mtl_id, std::vector<openvdb::Vec3I>{});

  it = _mtl_texcoord_map.find(mtl_id);
  auto &g = it->second;
  g.emplace_back(f);
}

void mesh_data::add_mtl_texcoords(int mtl_id,
                                  const std::vector<openvdb::Vec3I> &f) {
  auto it = _mtl_texcoord_map.find(mtl_id);
  if (it == _mtl_texcoord_map.end())
    _mtl_texcoord_map.emplace(mtl_id, std::vector<openvdb::Vec3I>{});

  auto &g = _mtl_texcoord_map.find(mtl_id)->second;
  g.insert(g.begin(), f.begin(), f.end());
}

void mesh_data::clear() {
  _vertices.clear();
  _faces.clear();
  _texcoords.clear();
  _normals.clear();
  _mtl_faces_map.clear();
  _mtl_normal_map.clear();
  _mtl_texcoord_map.clear();
  _smooth_faces_map.clear();
}
} // namespace flywave
