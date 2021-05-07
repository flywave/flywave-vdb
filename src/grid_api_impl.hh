#pragma once

#include "float_grid.hh"
#include "pixel_grid.hh"

#ifdef __cplusplus
extern "C" {
#endif

struct _vdb_float_grid_t {
  std::shared_ptr<flywave::vdb_float_grid> ptr;
};

struct _vdb_pixel_grid_t {
  std::shared_ptr<flywave::vdb_pixel_grid> ptr;
};

#ifdef __cplusplus
}
#endif
