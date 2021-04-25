#pragma once

#include <openvdb/tools/GridOperators.h>

namespace flywave {
namespace voxelize {

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
      : _grid(grid), inAccessor(grid->getConstAccessor()), _pixel_counts(0) {}

  void search(const std::vector<openvdb::Vec3d> &points,
              std::vector<closest_points_type> &distances) override {
    size_t i = 0;
    auto scale =
        _grid->transformPtr()->template map<openvdb::math::UniformScaleMap>();
    for (auto &point : points) {
      openvdb::Vec3f P =
          openvdb::math::CPT<openvdb::math::ScaleMap, openvdb::math::CD_2ND>::
              result<float>(*scale, inAccessor,
                            openvdb::math::Coord(point.x, point.y, point.z));
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
      : _closest_points_index(std::move(index)), _grid(grid),
        _accessor(grid->tree()) {}

  sampling_voxels extract(std::vector<openvdb::Vec3d> coords) {
    return sampling(std::move(coords));
  }

  size_t pixel_count() const { return _closest_points_index->pixel_count(); }

private:
  template <typename T> struct approx_value {
    inline T operator()(T value) const {
      T c = std::ceil(value);
      if (openvdb::math::isApproxEqual(float(tol), float(c - value)))
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
      auto index =
          openvdb::math::Coord(value._coord.x, value._coord.y, value._coord.z);
      auto val = _accessor.get_value(index);
      if (val._data._type == pixel_data::type_t::invalid)
        val = _accessor.get_value(openvdb::math::Coord(
            value._point.x, value._point.y, value._point.z));
#if false
      if (val._color.r <= 15 && val._color.g < 4 &&
          val._color.b < 4) {
        val._color = color4<uint8_t>(255, 0, 0, 255);
      }
#endif
      result.emplace_back(sampling_result<typename GridType::ValueType>{
          value._coord, value._point, val});
    }

    return std::move(result);
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
} // namespace voxelize
} // namespace flywave
