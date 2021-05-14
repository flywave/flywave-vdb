#pragma once

#include "mesh_data.hh"
#include "particle.hh"
#include "transformer.hh"
#include "trees.hh"
#include "vdb_utils.hh"

#include <openvdb/openvdb.h>

#include <string>
#include <vector>

namespace flywave {

class vdb_float_grid {
public:
  vdb_float_grid();
  vdb_float_grid(vdb_float_grid *grid);
  vdb_float_grid(float_grid::Ptr grid);

  ~vdb_float_grid();

  float_grid::Ptr grid();
  bool has_grid() const;

  int64_t get_memory_size() const;

  double get_volume() const;

  double get_area() const;

  bool is_sdf() const;

  bool is_empty() { return _grid == nullptr; }

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
  void smooth(int type, int iterations, int width, vdb_float_grid vMask,
              double min, double max, bool invert);

  void blend(vdb_float_grid bGrid, double bPosition, double bEnd);
  void blend(vdb_float_grid bGrid, double bPosition, double bEnd,
             vdb_float_grid vMask, double min, double max, bool invert);

  void closest_point(std::vector<openvdb::Vec3R> &points,
                     std::vector<float> &distances);

  openvdb::Vec3d bary_center();

  mesh_data display();

  void update_display();
  void update_display(double isovalue, double adaptivity);

  void rebuild(float iso, float exWidth, float inWidth);

  std::shared_ptr<vdb_float_grid> resample(float_grid::Ptr refGrid,
                                           float voxelSize, int curOrder,
                                           float tolerance, bool prune);

  template <typename GridTransformer>
  std::shared_ptr<vdb_float_grid> resample(GridTransformer &trans, int curOrder,
                                           float tolerance, bool prune) {
    vdb_grid_ptr outGrid = _grid->copyGridWithNewTree();

    if (curOrder == 0) {
      grid_resample_op<openvdb::tools::PointSampler, GridTransformer> op(
          outGrid, trans);
      op(*_grid);
    } else if (curOrder == 1) {
      grid_resample_op<openvdb::tools::BoxSampler, GridTransformer> op(outGrid,
                                                                       trans);
      op(*_grid);
    } else if (curOrder == 2) {
      grid_resample_op<openvdb::tools::QuadraticSampler, GridTransformer> op(
          outGrid, trans);
      op(*_grid);
    }

    if (prune)
      outGrid->pruneGrid(tolerance);

    return std::make_shared<vdb_float_grid>(vdb_grid_cast<float_grid>(outGrid));
  }

  void set(const int i, const int j, const int k, const float &v);

  float operator()(const int i, const int j, const int k) const;

  float operator()(const float i, const float j, const float k) const;

  float *get_mesh_vertices();

  int *get_mesh_faces();

  int get_vertex_count();

  int get_face_count();

  double calc_positive_density() const;

  template <typename GridTypeListT, typename OpT> bool apply(OpT &op) const {
    return has_grid() ? _grid->apply<GridTypeListT>(op) : false;
  }

private:
  float_grid::Ptr _grid;
  mesh_data _display;
  int _face_count;
  int _vertex_count;
};

} // namespace flywave
