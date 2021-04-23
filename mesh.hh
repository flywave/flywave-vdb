#pragma once

#include <openvdb/openvdb.h>

#include <vector>

namespace flywave {

class vdb_mesh {
public:
  vdb_mesh();
  ~vdb_mesh();

  vdb_mesh duplicate();

  bool is_valid();

  std::vector<openvdb::Vec3s> vertices();
  std::vector<openvdb::Vec4I> faces();

  void add_vertice(openvdb::Vec3s v);
  void add_vertice(std::vector<openvdb::Vec3s> v);

  void add_face(openvdb::Vec4I f);
  void add_face(std::vector<openvdb::Vec4I> f);

  void clear();

private:
  std::vector<openvdb::Vec3s> _vertices;
  std::vector<openvdb::Vec4I> _faces;
};
} // namespace flywave