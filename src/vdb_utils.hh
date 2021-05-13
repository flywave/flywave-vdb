#pragma once

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/Statistics.h>

#include <memory>

namespace flywave {

static inline openvdb::math::Vec3d
vdb_translation(const openvdb::math::Transform &xform) {
  return xform.baseMap()->getAffineMap()->getMat4().getTranslation();
}

template <typename GridType>
static inline typename GridType::ValueType
vdb_double_to_grid_value(double val) {
  using ValueT = typename GridType::ValueType;
  return ValueT(openvdb::zeroVal<ValueT>() + val);
}

template <typename GridType>
static double vdb_sample_grid(const GridType &grid, const openvdb::Vec3d &pos) {
  const openvdb::math::Transform &xform = grid.transform();
  openvdb::math::Vec3d vpos;
  typename GridType::ValueType value;

  vpos = openvdb::math::Vec3d(pos.x(), pos.y(), pos.z());
  vpos = xform.worldToIndex(vpos);

  openvdb::tools::BoxSampler::sample(grid.tree(), vpos, value);

  double result = value;

  return result;
}

template <typename GridType>
static double vdb_sample_bool_grid(const GridType &grid,
                                   const openvdb::Vec3d &pos) {
  const openvdb::math::Transform &xform = grid.transform();
  openvdb::math::Vec3d vpos;
  typename GridType::ValueType value;

  vpos = openvdb::math::Vec3d(pos.x(), pos.y(), pos.z());
  vpos = xform.worldToIndex(vpos);

  openvdb::tools::PointSampler::sample(grid.tree(), vpos, value);

  double result = value;

  return result;
}

template <typename GridType>
static openvdb::Vec3d vdb_sample_grid_v3(const GridType &grid,
                                         const openvdb::Vec3d &pos) {
  const openvdb::math::Transform &xform = grid.transform();
  openvdb::math::Vec3d vpos;
  typename GridType::ValueType value;

  vpos = openvdb::math::Vec3d(pos.x(), pos.y(), pos.z());
  vpos = xform.worldToIndex(vpos);

  openvdb::tools::BoxSampler::sample(grid.tree(), vpos, value);

  openvdb::Vec3d result;
  result.x() = double(value[0]);
  result.y() = double(value[1]);
  result.z() = double(value[2]);

  return result;
}

template <typename GridType, typename T, typename IDX>
static void vdb_sample_grid_many(const GridType &grid, T *f, int stride,
                                 const IDX *pos, int num) {
  typename GridType::ConstAccessor accessor = grid.getAccessor();

  const openvdb::math::Transform &xform = grid.transform();
  openvdb::math::Vec3d vpos;
  typename GridType::ValueType value;

  for (int i = 0; i < num; i++) {
    vpos = openvdb::math::Vec3d(pos[i].x(), pos[i].y(), pos[i].z());
    vpos = xform.worldToIndex(vpos);

    openvdb::tools::BoxSampler::sample(accessor, vpos, value);

    *f = T(value);
    f += stride;
  }
}

template <typename GridType, typename T, typename IDX>
static void vdb_sample_bool_grid_many(const GridType &grid, T *f, int stride,
                                      const IDX *pos, int num) {
  typename GridType::ConstAccessor accessor = grid.getAccessor();

  const openvdb::math::Transform &xform = grid.transform();
  openvdb::math::Vec3d vpos;
  typename GridType::ValueType value;

  for (int i = 0; i < num; i++) {
    vpos = openvdb::math::Vec3d(pos[i].x(), pos[i].y(), pos[i].z());
    vpos = xform.worldToIndex(vpos);

    openvdb::tools::PointSampler::sample(accessor, vpos, value);

    *f = T(value);
    f += stride;
  }
}

template <typename GridType, typename T, typename IDX>
static void vdb_sample_vec_grid_many(const GridType &grid, T *f, int stride,
                                     const IDX *pos, int num) {
  typename GridType::ConstAccessor accessor = grid.getAccessor();

  const openvdb::math::Transform &xform = grid.transform();
  openvdb::math::Vec3d vpos;
  typename GridType::ValueType value;

  for (int i = 0; i < num; i++) {
    vpos = openvdb::math::Vec3d(pos[i].x(), pos[i].y(), pos[i].z());
    vpos = xform.worldToIndex(vpos);

    openvdb::tools::BoxSampler::sample(accessor, vpos, value);

    f->x() = value[0];
    f->y() = value[1];
    f->z() = value[2];
    f += stride;
  }
}

template <typename GridType>
static double vdb_sample_index(const GridType &grid, int ix, int iy, int iz) {
  openvdb::math::Coord xyz;
  typename GridType::ValueType value;

  xyz = openvdb::math::Coord(ix, iy, iz);

  value = grid.tree().getValue(xyz);

  double result = value;

  return result;
}

template <typename GridType>
static openvdb::Vec3d vdb_sample_index_v3(const GridType &grid, int ix, int iy,
                                          int iz) {
  openvdb::math::Coord xyz;
  typename GridType::ValueType value;

  xyz = openvdb::math::Coord(ix, iy, iz);

  value = grid.tree().getValue(xyz);

  openvdb::Vec3d result;

  result.x() = double(value[0]);
  result.y() = double(value[1]);
  result.z() = double(value[2]);

  return result;
}

template <typename GridType, typename T, typename IDX>
static void vdb_sample_index_many(const GridType &grid, T *f, int stride,
                                  const IDX *ix, const IDX *iy, const IDX *iz,
                                  int num) {
  typename GridType::ConstAccessor accessor = grid.getAccessor();

  openvdb::math::Coord xyz;
  typename GridType::ValueType value;

  for (int i = 0; i < num; i++) {
    xyz = openvdb::math::Coord(ix[i], iy[i], iz[i]);

    value = accessor.getValue(xyz);

    *f = T(value);
    f += stride;
  }
}

template <typename GridType, typename T, typename IDX>
static void vdb_sample_vec_index_many(const GridType &grid, T *f, int stride,
                                      const IDX *ix, const IDX *iy,
                                      const IDX *iz, int num) {
  typename GridType::ConstAccessor accessor = grid.getAccessor();

  openvdb::math::Coord xyz;
  typename GridType::ValueType value;

  for (int i = 0; i < num; i++) {
    xyz = openvdb::math::Coord(ix[i], iy[i], iz[i]);

    value = accessor.getValue(xyz);

    f->x() = value[0];
    f->y() = value[1];
    f->z() = value[2];
    f += stride;
  }
}

template <typename GridType>
static void vdb_calc_min(GridType &grid, double &result) {
  auto val = openvdb::tools::extrema(grid.cbeginValueOn());
  result = val.min();
}

template <typename GridType>
static void vdb_calc_max(GridType &grid, double &result) {
  auto val = openvdb::tools::extrema(grid.cbeginValueOn());
  result = val.max();
}

template <typename GridType>
static void vdb_calc_avg(GridType &grid, double &result) {
  auto val = openvdb::tools::statistics(grid.cbeginValueOn());
  result = val.avg();
}

template <typename GridTypeA, typename GridTypeB>
static void vdb_intersect(GridTypeA &grid_a, const GridTypeB &grid_b) {
  typename GridTypeA::Accessor access_a = grid_a.getAccessor();
  typename GridTypeB::ConstAccessor access_b = grid_b.getAccessor();

  for (typename GridTypeA::ValueOnCIter iter = grid_a.cbeginValueOn(); iter;
       ++iter) {
    openvdb::CoordBBox bbox = iter.getBoundingBox();
    for (int k = bbox.min().z(); k <= bbox.max().z(); k++) {
      for (int j = bbox.min().y(); j <= bbox.max().y(); j++) {
        for (int i = bbox.min().x(); i <= bbox.max().x(); i++) {
          openvdb::Coord coord(i, j, k);
          if (!access_b.isValueOn(coord)) {
            access_a.setValue(coord, grid_a.background());
            access_a.setValueOff(coord);
          }
        }
      }
    }
  }
}

template <typename GridType> class vdb_inactive_to_background {
public:
  typedef typename GridType::ValueOffIter Iterator;
  typedef typename GridType::ValueType ValueType;

  vdb_inactive_to_background(const GridType &grid) {
    background = grid.background();
  }

  inline void operator()(const Iterator &iter) const {
    iter.setValue(background);
  }

private:
  ValueType background;
};

enum ActivateOperation {
  ACTIVATE_UNION,     // Activate anything in source
  ACTIVATE_INTERSECT, // Deactivate anything not in source
  ACTIVATE_SUBTRACT,  // Deactivate anything in source
  ACTIVATE_COPY       // Set our activation to match source
};

template <typename GridType>
static void vdb_activate_bbox(GridType &grid, const openvdb::CoordBBox &bbox,
                              bool setvalue, double value,
                              ActivateOperation operation, bool doclip,
                              const openvdb::CoordBBox &clipbox) {
  typename GridType::Accessor access = grid.getAccessor();

  switch (operation) {
  case ACTIVATE_UNION: // Union
    if (doclip) {
      openvdb::CoordBBox clipped = bbox;
      clipped = bbox;
      clipped.min().maxComponent(clipbox.min());
      clipped.max().minComponent(clipbox.max());

      vdb_activate_bbox(grid, clipped, setvalue, value, operation, false,
                        clipped);
      break;
    }
    if (setvalue) {
      grid.fill(bbox, vdb_double_to_grid_value<GridType>(value),
                /*active*/ true);
    } else {
      openvdb::MaskGrid mask(false);
      mask.denseFill(bbox, true, true);
      grid.topologyUnion(mask);
    }
    break;
  case ACTIVATE_INTERSECT: // Intersect
  {
    openvdb::MaskGrid mask(false);
    mask.fill(bbox, true, true);
    grid.topologyIntersection(mask);
    vdb_inactive_to_background<GridType> bgop(grid);
    openvdb::tools::foreach (grid.beginValueOff(), bgop);
  } break;
  case ACTIVATE_SUBTRACT: // Difference
    grid.fill(bbox, grid.background(), /*active*/ false);
    break;
  case ACTIVATE_COPY: // Copy
    vdb_activate_bbox(grid, bbox, setvalue, value, ACTIVATE_INTERSECT, doclip,
                      clipbox);
    vdb_activate_bbox(grid, bbox, setvalue, value, ACTIVATE_UNION, doclip,
                      clipbox);
    break;
  }
}

template <typename GridTypeA, typename GridTypeB>
static void vdb_union(GridTypeA &grid_a, const GridTypeB &grid_b, bool setvalue,
                      double value, bool doclip,
                      const openvdb::CoordBBox &clipbox) {
  typename GridTypeA::Accessor access_a = grid_a.getAccessor();
  typename GridTypeB::ConstAccessor access_b = grid_b.getAccessor();

  if (!doclip && !setvalue) {
    grid_a.tree().topologyUnion(grid_b.tree());
    return;
  }

  for (typename GridTypeB::ValueOnCIter iter = grid_b.cbeginValueOn(); iter;
       ++iter) {
    openvdb::CoordBBox bbox = iter.getBoundingBox();
    if (doclip) {
      bbox.min().maxComponent(clipbox.min());
      bbox.max().minComponent(clipbox.max());
    }

    for (int k = bbox.min().z(); k <= bbox.max().z(); k++) {
      for (int j = bbox.min().y(); j <= bbox.max().y(); j++) {
        for (int i = bbox.min().x(); i <= bbox.max().x(); i++) {
          openvdb::Coord coord(i, j, k);
          if (setvalue) {
            access_a.setValue(coord,
                              vdb_double_to_grid_value<GridTypeA>(value));
          } else {
            access_a.setValueOn(coord);
          }
        }
      }
    }
  }
}

template <typename GridTypeA, typename GridTypeB>
static void vdb_difference(GridTypeA &grid_a, const GridTypeB &grid_b) {
  typename GridTypeA::Accessor access_a = grid_a.getAccessor();
  typename GridTypeB::ConstAccessor access_b = grid_b.getAccessor();

  for (typename GridTypeA::ValueOnCIter iter = grid_a.cbeginValueOn(); iter;
       ++iter) {
    openvdb::CoordBBox bbox = iter.getBoundingBox();
    for (int k = bbox.min().z(); k <= bbox.max().z(); k++) {
      for (int j = bbox.min().y(); j <= bbox.max().y(); j++) {
        for (int i = bbox.min().x(); i <= bbox.max().x(); i++) {
          openvdb::Coord coord(i, j, k);
          if (access_b.isValueOn(coord)) {
            access_a.setValue(coord, grid_a.background());
            access_a.setValueOff(coord);
          }
        }
      }
    }
  }
}

template <typename GridType>
static void vdb_sum_pos_density(const GridType &grid, double &sum) {
  sum = 0;
  for (typename GridType::ValueOnCIter iter = grid.cbeginValueOn(); iter;
       ++iter) {
    double value = *iter;
    if (value > 0) {
      if (iter.isTileValue())
        sum += value * iter.getVoxelCount();
      else
        sum += value;
    }
  }
}

} // namespace flywave
