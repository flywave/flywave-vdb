#pragma once

#include "float_grid.hh"
#include "mesh_data.hh"
#include "particle.hh"
#include "trees.hh"

#include <openvdb/openvdb.h>

#include <string>
#include <vector>

namespace flywave {

class vdb_pixel_grid {
public:
  vdb_pixel_grid();
  vdb_pixel_grid(vdb_pixel_grid *grid);
  vdb_pixel_grid(pixel_grid::Ptr grid);
  ~vdb_pixel_grid();

  pixel_grid::Ptr grid();
  bool has_grid() const;

  int64_t get_memory_size() const;

  double get_volume() const;
  double get_area() const;

  bool is_sdf() const;

  bool is_empty() { return _grid == nullptr; }

  void paint_texture(pixel_grid::Ptr texvol);

  bool read(const char *vFile);
  bool write(const char *vFile);

  void transform(openvdb::Mat4d xform);

  openvdb::Vec3d bary_center();

  void set(const int i, const int j, const int k, const pixel &v);

  pixel operator()(const int i, const int j, const int k) const;

  pixel operator()(const float i, const float j, const float k) const;

  double calc_positive_density() const;

  void sparse_fill(const vdb::CoordBBox &box, const pixel &v,
                   bool active = true);

  void fill(const vdb::CoordBBox &box, const pixel &v, bool active = true);

  void dense_fill(const vdb::CoordBBox &box, const pixel &v,
                  bool active = true);

  template <typename GridTypeListT, typename OpT> bool apply(OpT &op) const {
    return has_grid() ? _grid->apply<GridTypeListT>(op) : false;
  }

private:
  pixel_grid::Ptr _grid;
};

} // namespace flywave
