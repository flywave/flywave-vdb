#pragma once

#include <eigen3/Eigen/Geometry>

#include <openvdb/Types.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Morphology.h>

namespace flywave {

class mapping {
public:
  using PointT = Eigen::Matrix<double, 3, 1>;
  using PointCloudT = std::vector<Eigen::Matrix<double, 3, 1>>;

  using RayT = openvdb::math::Ray<double>;
  using Vec3T = RayT::Vec3Type;
  using DDAT = openvdb::math::DDA<RayT, 0>;

  using GridT = openvdb::FloatGrid;

  struct Config {
    double max_range;
    double prob_hit;
    double prob_miss;
    double prob_thres_min;
    double prob_thres_max;
  };

  mapping() = delete;
  mapping(const mapping &) = delete;
  mapping &operator=(const mapping &) = delete;

  mapping(const double resolution);
  virtual ~mapping(){};

  GridT::Ptr create_map(double resolution);

  void reset_map();

  bool insert_point_cloud(const PointCloudT &cloud,
                          const Eigen::Matrix<double, 3, 1> &origin);

  GridT::Ptr get_map() const { return _grid; }

  void set_config(const Config config);

private:
  GridT::Ptr _grid;

  double _max_range;

  double _resolution;

  double _logodds_hit;

  double _logodds_miss;

  double _logodds_thres_min;

  double _logodds_thres_max;

  bool _config_set;
};
} // namespace flywave