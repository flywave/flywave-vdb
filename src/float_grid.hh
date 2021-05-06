#pragma once

#include "particle.hh"
#include "mesh_data.hh"
#include "trees.hh"

#include <openvdb/openvdb.h>

#include <string>
#include <vector>

namespace flywave {

class vdb_float_grid {
public:
  vdb_float_grid();
  vdb_float_grid(vdb_float_grid *grid);
  ~vdb_float_grid();

  float_grid::Ptr grid();

  bool read(const char *vFile);
  bool write(const char *vFile);

  bool create_from_mesh(mesh_data vMesh, double voxelSize, double bandwidth);
  bool create_from_points(vdb_particle vPoints, double voxelSize,
                          double bandwidth);

  void transform(openvdb::Mat4d xform);

  void boolean_union(vdb_float_grid vAdd);
  void boolean_intersection(vdb_float_grid vIntersect);
  void boolean_difference(vdb_float_grid vSubtract);

  void offset(double amount);
  void offset(double amount, vdb_float_grid vMask, double min, double max,
              bool invert);

  void smooth(int type, int iterations, int width);
  void smooth(int type, int iterations, int width, vdb_float_grid vMask, double min,
              double max, bool invert);

  void blend(vdb_float_grid bGrid, double bPosition, double bEnd);
  void blend(vdb_float_grid bGrid, double bPosition, double bEnd, vdb_float_grid vMask,
             double min, double max, bool invert);

  void closest_point(std::vector<openvdb::Vec3R> &points,
                     std::vector<float> &distances);

  mesh_data display();

  void update_display();
  void update_display(double isovalue, double adaptivity);

  void rebuild(float iso, float exWidth, float inWidth);

  void set(const int i, const int j, const int k, const float &v);

  float operator()(const int i, const int j, const int k) const;

  float operator()(const float i, const float j, const float k) const;

  float *get_mesh_vertices();

  int *get_mesh_faces();

  int get_vertex_count();

  int get_face_count();

private:
  float_grid::Ptr _grid;
  mesh_data _display;
  int _face_count;
  int _vertex_count;
};

} // namespace flywave
