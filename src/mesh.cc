#include "mesh.hh"

namespace flywave {

vdb_mesh::vdb_mesh() {
  _vertices.clear();
  _faces.clear();
}

vdb_mesh vdb_mesh::duplicate() {
  vdb_mesh mesh;
  mesh.add_vertice(_vertices);
  mesh.add_face(_faces);
  return mesh;
}

vdb_mesh::~vdb_mesh() {}

bool vdb_mesh::is_valid() {
  if (_faces.size() > 0 && _vertices.size() > 0) {
    return true;
  }
  return false;
}

std::vector<openvdb::Vec3s> vdb_mesh::vertices() { return _vertices; }

std::vector<openvdb::Vec4I> vdb_mesh::faces() { return _faces; }

void vdb_mesh::add_vertice(openvdb::Vec3s v) { _vertices.push_back(v); }

void vdb_mesh::add_vertice(std::vector<openvdb::Vec3s> v) {
  _vertices.insert(_vertices.end(), v.begin(), v.end());
}

void vdb_mesh::add_face(openvdb::Vec4I f) { _faces.push_back(f); }

void vdb_mesh::add_face(std::vector<openvdb::Vec4I> f) {
  _faces.insert(_faces.end(), f.begin(), f.end());
}

void vdb_mesh::clear() {
  _vertices.clear();
  _faces.clear();
}
} // namespace flywave