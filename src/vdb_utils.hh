#pragma once

#include "types.hh"

#include <memory>
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/LevelSetMeasure.h>
#include <openvdb/tools/Statistics.h>

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

template <typename GridType>
static void vdb_calc_volume(const GridType &grid, double &volume) {
  bool calculated = false;
  if (grid.getGridClass() == openvdb::GRID_LEVEL_SET) {
    try {
      volume = openvdb::tools::levelSetVolume(grid);
      calculated = true;
    } catch (std::exception & /*e*/) {
      // do nothing
    }
  }

  if (!calculated) {
    const openvdb::Vec3d size = grid.voxelSize();
    volume = size[0] * size[1] * size[2] * grid.activeVoxelCount();
  }
}

template <typename GridType>
static void vdb_calc_area(const GridType &grid, double &area) {
  bool calculated = false;
  if (grid.getGridClass() == openvdb::GRID_LEVEL_SET) {
    try {
      area = openvdb::tools::levelSetArea(grid);
      calculated = true;
    } catch (std::exception & /*e*/) {
      // do nothing
    }
  }

  if (!calculated) {
    using LeafIter = typename GridType::TreeType::LeafCIter;
    using VoxelIter = typename GridType::TreeType::LeafNodeType::ValueOnCIter;
    using openvdb::Coord;
    const Coord normals[] = {Coord(0, 0, -1), Coord(0, 0, 1),  Coord(-1, 0, 0),
                             Coord(1, 0, 0),  Coord(0, -1, 0), Coord(0, 1, 0)};

    openvdb::Vec3d voxel_size = grid.voxelSize();
    const double areas[] = {double(voxel_size.x() * voxel_size.y()),
                            double(voxel_size.x() * voxel_size.y()),
                            double(voxel_size.y() * voxel_size.z()),
                            double(voxel_size.y() * voxel_size.z()),
                            double(voxel_size.z() * voxel_size.x()),
                            double(voxel_size.z() * voxel_size.x())};
    area = 0;
    for (LeafIter leaf = grid.tree().cbeginLeaf(); leaf; ++leaf) {
      for (VoxelIter iter = leaf->cbeginValueOn(); iter; ++iter) {
        for (int i = 0; i < 6; i++)
          if (!grid.tree().isValueOn(iter.getCoord() + normals[i]))
            area += areas[i];
      }
    }
  }
}

namespace {

template <typename ValueType>
inline typename std::enable_if<!openvdb::VecTraits<ValueType>::IsVec,
                               ValueType>::type
convert_value(const openvdb::Vec3R &val) {
  return ValueType(val[0]);
}

template <typename ValueType>
inline typename std::enable_if<openvdb::VecTraits<ValueType>::IsVec &&
                                   openvdb::VecTraits<ValueType>::Size == 2,
                               ValueType>::type
convert_value(const openvdb::Vec3R &val) {
  using ElemType = typename openvdb::VecTraits<ValueType>::ElementType;
  return ValueType(ElemType(val[0]), ElemType(val[1]));
}

template <typename ValueType>
inline typename std::enable_if<openvdb::VecTraits<ValueType>::IsVec &&
                                   openvdb::VecTraits<ValueType>::Size == 3,
                               ValueType>::type
convert_value(const openvdb::Vec3R &val) {
  using ElemType = typename openvdb::VecTraits<ValueType>::ElementType;
  return ValueType(ElemType(val[0]), ElemType(val[1]), ElemType(val[2]));
}

template <typename ValueType>
inline typename std::enable_if<openvdb::VecTraits<ValueType>::IsVec &&
                                   openvdb::VecTraits<ValueType>::Size == 4,
                               ValueType>::type
convert_value(const openvdb::Vec3R &val) {
  using ElemType = typename openvdb::VecTraits<ValueType>::ElementType;
  return ValueType(ElemType(val[0]), ElemType(val[1]), ElemType(val[2]),
                   ElemType(1.0));
}

} // namespace

struct vdb_fill_op {
  const openvdb::CoordBBox indexBBox;
  const openvdb::BBoxd worldBBox;
  const openvdb::Vec3R value;
  const bool active, sparse;

  vdb_fill_op(const openvdb::CoordBBox &b, const openvdb::Vec3R &val, bool on,
              bool sparse_)
      : indexBBox(b), value(val), active(on), sparse(sparse_) {}

  vdb_fill_op(const openvdb::BBoxd &b, const openvdb::Vec3R &val, bool on,
              bool sparse_)
      : worldBBox(b), value(val), active(on), sparse(sparse_) {}

  template <typename GridT> void operator()(GridT &grid) const {
    openvdb::CoordBBox bbox = indexBBox;
    if (worldBBox) {
      openvdb::math::Vec3d imin, imax;
      openvdb::math::calculateBounds(grid.constTransform(), worldBBox.min(),
                                     worldBBox.max(), imin, imax);
      bbox.reset(openvdb::Coord::floor(imin), openvdb::Coord::ceil(imax));
    }
    using ValueT = typename GridT::ValueType;
    if (sparse) {
      grid.sparseFill(bbox, convert_value<ValueT>(value), active);
    } else {
      grid.denseFill(bbox, convert_value<ValueT>(value), active);
    }
  }
};

template <typename Sampler> class grid_transform_op {
public:
  grid_transform_op(vdb_grid_ptr &outGrid,
                    const openvdb::tools::GridTransformer &t)
      : mOutGrid(outGrid), mTransformer(t) {}

  template <typename GridType> void operator()(const GridType &inGrid) {
    typename GridType::Ptr outGrid = openvdb::gridPtrCast<GridType>(mOutGrid);
    mTransformer.transformGrid<Sampler, GridType>(inGrid, *outGrid);
  }

private:
  vdb_grid_ptr mOutGrid;
  openvdb::tools::GridTransformer mTransformer;
};

template <typename Sampler, typename TransformerType> class grid_resample_op {
public:
  grid_resample_op(vdb_grid_ptr &outGrid, const TransformerType &t)
      : mOutGrid(outGrid), mTransformer(t) {}

  template <typename GridType> void operator()(const GridType &inGrid) {
    typename GridType::Ptr outGrid = openvdb::gridPtrCast<GridType>(mOutGrid);
    openvdb::tools::GridResampler resampler;
    resampler.transformGrid<Sampler>(mTransformer, inGrid, *outGrid);
  }

private:
  vdb_grid_ptr mOutGrid;
  const TransformerType &mTransformer;
};

template <typename Sampler> class grid_resample_to_match_op {
public:
  grid_resample_to_match_op(vdb_grid_ptr outGrid) : mOutGrid(outGrid) {}

  template <typename GridType> void operator()(const GridType &inGrid) {
    typename GridType::Ptr outGrid = openvdb::gridPtrCast<GridType>(mOutGrid);
    openvdb::util::NullInterrupter interrupter;
    openvdb::tools::resampleToMatch<Sampler>(inGrid, *outGrid, interrupter);
  }

private:
  vdb_grid_ptr mOutGrid;
};

template <typename GridT>
bool eval_grid_bbox(GridT grid, openvdb::Vec3R corners[8],
                    bool expandHalfVoxel) {
  if (grid.activeVoxelCount() == 0)
    return false;

  openvdb::CoordBBox activeBBox = grid.evalActiveVoxelBoundingBox();
  if (!activeBBox)
    return false;

  openvdb::BBoxd voxelBBox(activeBBox.min().asVec3d(),
                           activeBBox.max().asVec3d());
  if (expandHalfVoxel) {
    voxelBBox.min() -= openvdb::Vec3d(0.5);
    voxelBBox.max() += openvdb::Vec3d(0.5);
  }

  openvdb::Vec3R bbox[8];
  bbox[0] = voxelBBox.min();
  bbox[1].init(voxelBBox.min()[0], voxelBBox.min()[1], voxelBBox.max()[2]);
  bbox[2].init(voxelBBox.max()[0], voxelBBox.min()[1], voxelBBox.max()[2]);
  bbox[3].init(voxelBBox.max()[0], voxelBBox.min()[1], voxelBBox.min()[2]);
  bbox[4].init(voxelBBox.min()[0], voxelBBox.max()[1], voxelBBox.min()[2]);
  bbox[5].init(voxelBBox.min()[0], voxelBBox.max()[1], voxelBBox.max()[2]);
  bbox[6] = voxelBBox.max();
  bbox[7].init(voxelBBox.max()[0], voxelBBox.max()[1], voxelBBox.min()[2]);

  const openvdb::math::Transform &xform = grid.transform();
  bbox[0] = xform.indexToWorld(bbox[0]);
  bbox[1] = xform.indexToWorld(bbox[1]);
  bbox[2] = xform.indexToWorld(bbox[2]);
  bbox[3] = xform.indexToWorld(bbox[3]);
  bbox[4] = xform.indexToWorld(bbox[4]);
  bbox[5] = xform.indexToWorld(bbox[5]);
  bbox[6] = xform.indexToWorld(bbox[6]);
  bbox[7] = xform.indexToWorld(bbox[7]);

  for (size_t i = 0; i < 8; ++i) {
    corners[i] = bbox[i];
  }

  return true;
}

inline openvdb::CoordBBox make_coord_bbox(const openvdb::BBoxd &b,
                                          const openvdb::math::Transform &t) {
  openvdb::Vec3d minWS, maxWS, minIS, maxIS;

  minWS[0] = double(b.min().x());
  minWS[1] = double(b.min().y());
  minWS[2] = double(b.min().z());

  maxWS[0] = double(b.max().x());
  maxWS[1] = double(b.max().y());
  maxWS[2] = double(b.max().z());

  openvdb::math::calculateBounds(t, minWS, maxWS, minIS, maxIS);

  openvdb::CoordBBox box;
  box.min() = openvdb::Coord::floor(minIS);
  box.max() = openvdb::Coord::ceil(maxIS);

  return box;
}

template <typename GridType>
inline const GridType *vdb_grid_cast(const openvdb::GridBase *grid) {
  return dynamic_cast<const GridType *>(grid);
}

template <typename GridType>
inline GridType *vdb_grid_cast(openvdb::GridBase *grid) {
  return dynamic_cast<GridType *>(grid);
}

template <typename GridType>
inline const GridType &vdb_grid_cast(const openvdb::GridBase &grid) {
  return *dynamic_cast<const GridType *>(&grid);
}

template <typename GridType>
inline GridType &vdb_grid_cast(openvdb::GridBase &grid) {
  return *dynamic_cast<GridType *>(&grid);
}

template <typename GridType>
inline typename GridType::ConstPtr
vdb_grid_cast(openvdb::GridBase::ConstPtr grid) {
  return openvdb::gridConstPtrCast<GridType>(grid);
}

template <typename GridType>
inline typename GridType::Ptr vdb_grid_cast(openvdb::GridBase::Ptr grid) {
  return openvdb::gridPtrCast<GridType>(grid);
}

} // namespace flywave
