
#include "voxel_pixel.hh"

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

#include <fstream>

#include <openvdb/io/Compression.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Clip.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/RayIntersector.h>

namespace flywave {

template <typename RealType> struct ValueRange {
public:
  ValueRange()
      : _min(std::numeric_limits<RealType>::max()),
        _max(std::numeric_limits<RealType>::min()) {}
  ValueRange(RealType min_, RealType max_) : _min(min_), _max(max_) {}

  RealType getMin() const { return _min; }
  RealType getMax() const { return _max; }

  void addValue(RealType value) {
    _min = std::min(_min, value);
    _max = std::max(_max, value);
  }

private:
  RealType _min, _max;
};

typedef ValueRange<float> FloatRange;

enum class FilterMode { BOX, MULTIRES, AUTO };

template <typename T> struct identity { using type = T; };

template <typename T>
inline T unlerp(typename identity<T>::type a, typename identity<T>::type b,
                T x) {
  return (x - a) / (b - a);
}

inline openvdb::CoordBBox
getIndexSpaceBoundingBox(const openvdb::GridBase &grid) {
  try {
    const auto file_bbox_min =
        openvdb::Coord(grid.metaValue<openvdb::Vec3i>("file_bbox_min"));
    if (file_bbox_min.x() == std::numeric_limits<int>::max() ||
        file_bbox_min.y() == std::numeric_limits<int>::max() ||
        file_bbox_min.z() == std::numeric_limits<int>::max()) {
      return {};
    }
    const auto file_bbox_max =
        openvdb::Coord(grid.metaValue<openvdb::Vec3i>("file_bbox_max"));

    if (file_bbox_max.x() == std::numeric_limits<int>::min() ||
        file_bbox_max.y() == std::numeric_limits<int>::min() ||
        file_bbox_max.z() == std::numeric_limits<int>::min()) {
      return {};
    }

    return {file_bbox_min, file_bbox_max};
  } catch (openvdb::Exception e) {
    return {};
  }
}

template <typename SamplingFunc, typename RealType>
bool sampleVolume(const openvdb::Coord &extents, SamplingFunc sampling_func,
                  FloatRange &out_value_range, RealType *out_samples) {
  const auto domain = openvdb::CoordBBox(openvdb::Coord(0, 0, 0),
                                         extents - openvdb::Coord(1, 1, 1));
  if (domain.empty()) {
    return false;
  }
  const auto num_voxels = domain.volume();

  typedef tbb::enumerable_thread_specific<FloatRange> PerThreadRange;
  PerThreadRange ranges;
  const openvdb::Vec3i stride = {1, extents.x(), extents.x() * extents.y()};
  tbb::atomic<bool> cancelled;
  cancelled = false;
  tbb::parallel_for(domain, [&sampling_func, &stride, &ranges, out_samples,
                             &cancelled](const openvdb::CoordBBox &bbox) {
    PerThreadRange::reference this_thread_range = ranges.local();
    for (auto z = bbox.min().z(); z <= bbox.max().z(); ++z) {
      for (auto y = bbox.min().y(); y <= bbox.max().y(); ++y) {
        for (auto x = bbox.min().x(); x <= bbox.max().x(); ++x) {
          const auto domain_index = openvdb::Vec3i(x, y, z);
          const auto linear_index = domain_index.dot(stride) * 4;
          const auto sample_value = sampling_func(domain_index);

          out_samples[linear_index + 0] = sample_value;
          out_samples[linear_index + 1] = sample_value;
          out_samples[linear_index + 2] = sample_value;
          out_samples[linear_index + 3] = sample_value;
          this_thread_range.addValue(sample_value);
        }
      }
    }
  });

  out_value_range = FloatRange();
  for (const FloatRange &per_thread_range : ranges) {
    out_value_range.addValue(per_thread_range.getMin());
    out_value_range.addValue(per_thread_range.getMax());
  }

  int size = num_voxels * 4;
  typedef tbb::blocked_range<size_t> tbb_range;
  tbb::parallel_for(tbb_range(0, size),
                    [out_samples, &out_value_range](const tbb_range &range) {
                      for (auto i = range.begin(); i < range.end(); ++i) {
                        out_samples[i] =
                            unlerp(out_value_range.getMin(),
                                   out_value_range.getMax(), out_samples[i]);
                      }
                    });
}

template <typename RealType>
bool sampleGrid(const openvdb::FloatGrid &grid,
                const openvdb::Coord &sampling_extents, FloatRange &value_range,
                openvdb::Vec3d &scale, RealType *out_data) {
  assert(out_data);

  const auto grid_bbox_is = getIndexSpaceBoundingBox(grid);
  const auto bbox_world = grid.transform().indexToWorld(grid_bbox_is);

  scale = bbox_world.extents();

  if (grid_bbox_is.empty()) {
    return false;
  }

  const auto domain_extents = sampling_extents.asVec3d();
  openvdb::tools::GridSampler<openvdb::FloatGrid, openvdb::tools::BoxSampler>
      sampler(grid);

  auto sampling_func = [&sampler, &bbox_world, &domain_extents](
                           const openvdb::Vec3d &domain_index) -> RealType {
    const auto sample_pos_ws = bbox_world.min() + (domain_index + 0.5) /
                                                      domain_extents *
                                                      bbox_world.extents();
    return sampler.wsSample(sample_pos_ws);
  };

  sampleVolume(sampling_extents, sampling_func, value_range, out_data);
}

voxel_pixel::voxel_pixel(vertex_grid::Ptr vertex, pixel_grid::Ptr pixel,
                         vdb::math::Transform::Ptr res)
    : _resolution(res), _vertex(vertex), _pixel(pixel) {
  _vertex->setGridClass(openvdb::GRID_LEVEL_SET);
  _vertex->setTransform(_resolution);
}

bool voxel_pixel::ray_test(const vdb::math::Ray<double> &ray,
                           openvdb::Vec3d &p) {
  if (_vertex->empty())
    return false;

  openvdb::tools::LevelSetRayIntersector<vertex_grid> vray(*_vertex);

  return vray.intersectsWS(ray, p);
}

void voxel_pixel::clear_unuse_materials() {
  std::map<material_id_t, bool> mapping;
  for (auto pt : _materials) {
    mapping.emplace(pt->_material_id, true);
  }

  auto iter = get_pixel_grid()->tree().beginValueOn();
  while (iter) {
    mapping[iter.getValue()._data._material_id] = false;
    ++iter;
  }

  for (auto pt : mapping) {
    if (pt.second == false)
      _materials.erase(_materials.begin() + pt.first);
  }
}

void voxel_pixel_intersection(voxel_pixel &tpot, voxel_pixel &spot) {
  openvdb::tools::csgIntersection(tpot.get_voxel_grid()->tree(),
                                  spot.get_voxel_grid()->tree(), true);
}

void voxel_pixel_union(voxel_pixel &tpot, voxel_pixel &spot) {
  openvdb::tools::csgUnion(tpot.get_voxel_grid()->tree(),
                           spot.get_voxel_grid()->tree(), true);
  tpot.get_pixel_grid()->tree().merge(spot.get_pixel_grid()->tree());

  auto iter = spot.get_voxel_grid()->tree().beginValueOn();
  auto paccess = tpot.get_pixel_grid()->getAccessor();
  auto vaccess = tpot.get_voxel_grid()->getAccessor();

  while (iter) {
    if (vaccess.isValueOn(iter.getCoord())) {
      paccess.setValue(iter.getCoord(), iter.getValue());
    } else {
      paccess.setValueOff(iter.getCoord());
    }
    ++iter;
  }

  openvdb::tools::prune(tpot.get_voxel_grid()->tree());
}

void voxel_pixel_difference(voxel_pixel &tpot, voxel_pixel &spot) {
  openvdb::tools::csgDifference(tpot.get_voxel_grid()->tree(),
                                spot.get_voxel_grid()->tree(), true);
}

void voxel_pixel::composite(voxel_pixel &pot, const composite_type &type) {
  _resolution = pot._resolution;

  if (!pot.is_empty()) {
    switch (type) {
    case composite_type::op_union:
      voxel_pixel_union(*this, pot);
      break;

    case composite_type::op_intersection:
      voxel_pixel_intersection(*this, pot);
      break;

    case composite_type::op_difference:
      voxel_pixel_difference(*this, pot);
      break;
    }
  }
  _vertex->setTransform(_resolution);
}

inline void
write_materials(std::ofstream &os,
                std::vector<std::shared_ptr<material_data>> &materials) {
  uint32_t si = materials.size();
  os.write(reinterpret_cast<const char *>(&si), sizeof(uint32_t));
  for (int i = 0; i < si; i++) {
    materials[i]->write(os);
  }
}

void voxel_pixel::write(const std::string &file) {
  std::ofstream os;

  os.open(file);

  if (os.is_open()) {
    _resolution->baseMap()->write(os);

    write_materials(os, _materials);

    _vertex->writeTopology(os);
    _vertex->writeBuffers(os);

    _pixel->writeTopology(os);
    _pixel->writeBuffers(os);
  }
  os.close();
}

inline void
read_materials(std::ifstream &is,
               std::vector<std::shared_ptr<material_data>> &materials) {
  uint32_t si;
  is.read(reinterpret_cast<char *>(&si), sizeof(uint32_t));
  materials.resize(si);
  for (int i = 0; i < si; i++) {
    materials[i]->read(is);
  }
}

void voxel_pixel::read(const std::string &file) {
  std::ifstream is;

  is.open(file);

  if (is.is_open()) {
    _resolution->baseMap()->read(is);

    read_materials(is, _materials);

    _vertex->readTopology(is);
    _vertex->readBuffers(is);

    _pixel->readTopology(is);
    _pixel->readBuffers(is);

    _vertex->setTransform(_resolution);
    _vertex->setGridClass(vdb::GRID_LEVEL_SET);
  }
  is.close();
}

} // namespace flywave
