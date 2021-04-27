#include "mapping.hh"

#include <iostream>

namespace flywave {

mapping::mapping(const double resolution)
    : _resolution(resolution), _config_set(false) {
  _grid = create_map(_resolution);
}

void mapping::reset_map() {
  _grid->clear();
  _grid = create_map(_resolution);
}

mapping::GridT::Ptr mapping::create_map(double resolution) {
  GridT::Ptr new_map = GridT::create(0.0);
  new_map->setTransform(
      openvdb::math::Transform::createLinearTransform(_resolution));
  new_map->setGridClass(openvdb::GRID_LEVEL_SET);
  return new_map;
}

bool mapping::insert_point_cloud(const PointCloudT &cloud,
                                 const Eigen::Matrix<double, 3, 1> &origin) {
  if (!_config_set) {
    std::cerr << "Map not properly configured. Did you call setConfig method?"
              << std::endl;
    return false;
  }

  RayT ray;
  DDAT dda;

  openvdb::Vec3d ray_origin_world(origin.x(), origin.y(), origin.z());
  const Vec3T ray_origin_index(_grid->worldToIndex(ray_origin_world));
  openvdb::Vec3d ray_end_world;
  openvdb::Vec3d ray_direction;
  bool max_range_ray;

  GridT::Accessor acc = _grid->getAccessor();

  GridT::Ptr temp_grid = GridT::create(0.0);
  GridT::Accessor temp_acc = temp_grid->getAccessor();

  openvdb::Vec3d x;
  double ray_length;

  for (const PointT &pt : cloud) {
    max_range_ray = false;
    ray_end_world = openvdb::Vec3d(pt.x(), pt.y(), pt.z());
    if (_max_range > 0.0 &&
        (ray_end_world - ray_origin_world).length() > _max_range) {
      ray_end_world = ray_origin_world +
                      (ray_end_world - ray_origin_world).unit() * _max_range;
      max_range_ray = true;
    }

    ray_direction = _grid->worldToIndex(ray_end_world - ray_origin_world);

    ray.setEye(ray_origin_index);
    ray.setDir(ray_direction);
    dda.init(ray);

    ray_length = ray_direction.length();
    ray_direction.normalize();

    double signed_distance = 1;
    while (signed_distance > 0) {
      x = openvdb::Vec3d(dda.voxel().x(), dda.voxel().y(), dda.voxel().z()) -
          ray_origin_index;
      signed_distance = ray_length - ray_direction.dot(x);
      temp_acc.setActiveState(dda.voxel(), true);
      dda.step();
    }

    if (!max_range_ray) {
      temp_acc.setValueOn(dda.voxel(), -1);
    }
  }

  auto miss = [&prob_miss = _logodds_miss,
               &prob_thres_min = _logodds_thres_min](float &voxel_value,
                                                     bool &active) {
    voxel_value += prob_miss;
    if (voxel_value < prob_thres_min) {
      active = false;
    }
  };

  auto hit = [&prob_hit = _logodds_hit, &prob_thres_max = _logodds_thres_max](
                 float &voxel_value, bool &active) {
    voxel_value += prob_hit;
    if (voxel_value > prob_thres_max) {
      active = true;
    }
  };

  for (GridT::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter) {
    if (*iter == -1) {
      acc.modifyValueAndActiveState(iter.getCoord(), hit);
    } else {
      acc.modifyValueAndActiveState(iter.getCoord(), miss);
    }
  }
  return true;
}

void mapping::set_config(const Config config) {
  if (config.prob_miss > 0.5) {
    std::cerr << "Probability for a miss should be below 0.5 but is "
              << config.prob_miss << std::endl;
    return;
  }
  if (config.prob_hit < 0.5) {
    std::cerr << "Probability for a hit should be above 0.5 but is "
              << config.prob_miss << std::endl;
    return;
  }

  if (config.max_range < 0.0) {
    std::cerr << "Max range of " << config.max_range
              << " invalid. Range cannot be negative." << config.prob_miss
              << std::endl;
    return;
  }
  _max_range = config.max_range;

  _logodds_miss = log(config.prob_miss) - log(1 - config.prob_miss);
  _logodds_hit = log(config.prob_hit) - log(1 - config.prob_hit);
  _logodds_thres_min =
      log(config.prob_thres_min) - log(1 - config.prob_thres_min);
  _logodds_thres_max =
      log(config.prob_thres_max) - log(1 - config.prob_thres_max);
  _config_set = true;
}
} // namespace flywave
