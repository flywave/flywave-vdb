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
  ~vdb_pixel_grid();

  pixel_grid::Ptr grid();

  bool read(const char *vFile);
  bool write(const char *vFile);

  void transform(openvdb::Mat4d xform);

  void set(const int i, const int j, const int k, const pixel &v);

  pixel operator()(const int i, const int j, const int k) const;

  pixel operator()(const float i, const float j, const float k) const;

private:
  pixel_grid::Ptr _grid;
};

} // namespace flywave
