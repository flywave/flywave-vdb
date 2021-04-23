#pragma once

#include "grid.hh"

#ifdef __cplusplus
extern "C" {
#endif

struct _vdb_grid_t {
  flywave::vdb_grid *ptr;
};

#ifdef __cplusplus
}
#endif
