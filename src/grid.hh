#pragma once

#include "particle.hh"
#include "vdb_mesh.hh"

#include <openvdb/openvdb.h>

#include <string>
#include <vector>

namespace flywave {

class vdb_grid {
public:
  vdb_grid();
  vdb_grid(vdb_grid *grid);
  ~vdb_grid();

  openvdb::FloatGrid::Ptr grid();

  bool read(const char *vFile);
  bool write(const char *vFile);

  bool create_from_mesh(vdb_mesh vMesh, double voxelSize, double bandwidth);
  bool create_from_points(vdb_particle vPoints, double voxelSize,
                          double bandwidth);

  void transform(openvdb::math::Mat4d xform);

  void boolean_union(vdb_grid vAdd);
  void boolean_intersection(vdb_grid vIntersect);
  void boolean_difference(vdb_grid vSubtract);

  void offset(double amount);
  void offset(double amount, vdb_grid vMask, double min, double max,
              bool invert);

  void smooth(int type, int iterations, int width);
  void smooth(int type, int iterations, int width, vdb_grid vMask, double min,
              double max, bool invert);

  void blend(vdb_grid bGrid, double bPosition, double bEnd);
  void blend(vdb_grid bGrid, double bPosition, double bEnd, vdb_grid vMask,
             double min, double max, bool invert);

  void closest_point(std::vector<openvdb::Vec3R> &points,
                     std::vector<float> &distances);

  vdb_mesh display();

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
  openvdb::FloatGrid::Ptr _grid;
  vdb_mesh _display;
  int _face_count;
  int _vertex_count;
};
} // namespace flywave
