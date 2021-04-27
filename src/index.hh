#pragma once

#include <openvdb/tools/GridOperators.h>

#include "pquery.hh"

namespace flywave {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

struct closest_points_type {
  float _distance;
  openvdb::Vec3d _coord;
  openvdb::Vec3d _point;
  bool b = false;
};

class closest_points_index {
public:
  virtual ~closest_points_index() = default;

  virtual void search(const std::vector<openvdb::Vec3d> &,
                      std::vector<closest_points_type> &) = 0;

  virtual size_t pixel_count() const = 0;
};

namespace {
inline double tac(openvdb::Vec3d t, size_t k) { return t[k]; }
} // namespace

template <typename GridT>
class near_voxels_index : public closest_points_index {
public:
  near_voxels_index(typename GridT::ConstPtr grid)
      : _pixel_counts(0),  _grid(grid), inAccessor(grid->getConstAccessor()) {}

  void search(const std::vector<openvdb::Vec3d> &points,
              std::vector<closest_points_type> &distances) override {
    size_t i = 0;
    auto scale =
        _grid->transformPtr()->template map<vdb::math::UniformScaleMap>();
    for (auto &point : points) {
      vdb::Coord coord(point.x(), point.y(), point.z());
      openvdb::Vec3f P =
          vdb::math::CPT<vdb::math::UniformScaleMap, vdb::math::CD_2ND>::
              result(*scale, inAccessor, coord);
      distances[i++] = closest_points_type{0, P, point, true};
    }
  }

  size_t pixel_count() const override { return _pixel_counts; }

private:
  size_t _pixel_counts;
  typename GridT::ConstPtr _grid;
  typename GridT::ConstAccessor inAccessor;
};

#define PI 3.141592653589793

template <typename ValueType> struct sampling_result {
  openvdb::Vec3d _coord;
  openvdb::Vec3d _tri_coord;
  ValueType _value;
  sampling_result() = default;
  sampling_result(const openvdb::Vec3d &c, const openvdb::Vec3d &tri_cood,
                  const ValueType &value)
      : _coord(c), _tri_coord(tri_cood), _value(value) {}
};

template <typename GridType> class triangle_range_query {
  using sampling_voxels =
      std::vector<sampling_result<typename GridType::ValueType>>;

public:
  triangle_range_query(std::unique_ptr<closest_points_index> index,
                       typename GridType::ConstPtr grid)
      : _closest_points_index(std::move(index)),
        _accessor(grid->tree()), _grid(grid) {}

  sampling_voxels extract(std::vector<openvdb::Vec3d> coords) {
    return sampling(std::move(coords));
  }

  size_t pixel_count() const { return _closest_points_index->pixel_count(); }

private:
  template <typename T> struct approx_value {
    inline T operator()(T value) const {
      T c = std::ceil(value);
      if (vdb::math::isApproxEqual(float(tol), float(c - value)))
        return c;
      return value;
    }

    T tol = tolerance<T>();
  };
  approx_value<double> _approx_value;

  sampling_voxels sampling(std::vector<openvdb::Vec3d> coords) {
    std::vector<closest_points_type> instanceRadius(coords.size());
    sampling_voxels result;

    _closest_points_index->search(coords, instanceRadius);

    for (auto &value : instanceRadius) {
      if (!value.b)
        continue;
      vdb::Coord index(value._point.x(), value._point.y(),
                                 value._point.z());
      vdb::Coord rindex(value._coord.x(), value._coord.y(),
                                  value._coord.z());
      pixel pix;
      if (_accessor.isValueOn(index))
        pix = _accessor.getValue(index);
      else if (_accessor.isValueOn(rindex)) {
        pix = _accessor.getValue(rindex);
      } else if (!seach_vertex_value(_accessor, index, pix)) {
        seach_vertex_value(_accessor, rindex, pix);
      }
      result.emplace_back(sampling_result<typename GridType::ValueType>{
          value._coord, value._point, pix});
    }

    return result;
  }

private:
  std::unique_ptr<closest_points_index> _closest_points_index;
  openvdb::tree::ValueAccessor<const typename GridType::TreeType> _accessor;
  typename GridType::ConstPtr _grid;
};

template <typename Grid, typename PixelGrid>
inline std::unique_ptr<triangle_range_query<PixelGrid>>
make_near_voxels_index(typename Grid::ConstPtr grid,
                       typename PixelGrid::ConstPtr pgrid) {
  return std::make_unique<triangle_range_query<PixelGrid>>(
      std::make_unique<near_voxels_index<Grid>>(grid), pgrid);
}

} // namespace flywave
