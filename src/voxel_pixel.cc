
#include "voxel_pixel.hh"
#include "bbox.hh"
#include "feature_meta_data.hh"
#include "material_meta_data.hh"

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

template <typename RealType> struct value_range {
public:
  value_range()
      : _min(std::numeric_limits<RealType>::max()),
        _max(std::numeric_limits<RealType>::min()) {}
  value_range(RealType min_, RealType max_) : _min(min_), _max(max_) {}

  RealType getMin() const { return _min; }
  RealType getMax() const { return _max; }

  void addValue(RealType value) {
    _min = std::min(_min, value);
    _max = std::max(_max, value);
  }

private:
  RealType _min, _max;
};

typedef value_range<float> float_range;

template <typename T> struct identity { using type = T; };

template <typename T>
inline T unlerp(typename identity<T>::type a, typename identity<T>::type b,
                T x) {
  return (x - a) / (b - a);
}

inline openvdb::CoordBBox
get_index_space_bounding_box(const openvdb::GridBase &grid) {
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
bool sample_volume(const openvdb::Coord &extents, SamplingFunc sampling_func,
                   float_range &out_value_range, RealType *out_samples) {
  const auto domain = openvdb::CoordBBox(openvdb::Coord(0, 0, 0),
                                         extents - openvdb::Coord(1, 1, 1));
  if (domain.empty()) {
    return false;
  }
  const auto num_voxels = domain.volume();

  typedef tbb::enumerable_thread_specific<float_range> PerThreadRange;
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

  out_value_range = float_range();
  for (const float_range &per_thread_range : ranges) {
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
bool sample_grid(const openvdb::FloatGrid &grid,
                 const openvdb::Coord &sampling_extents,
                 float_range &value_range, openvdb::Vec3d &scale,
                 RealType *out_data) {
  assert(out_data);

  const auto grid_bbox_is = get_index_space_bounding_box(grid);
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

  sample_volume(sampling_extents, sampling_func, value_range, out_data);
}

voxel_pixel::voxel_pixel(vertex_grid::Ptr vertex, pixel_grid::Ptr pixel,
                         vdb::math::Transform::Ptr res)
    : _resolution(res), _vertex(vertex), _pixel(pixel) {
  _vertex->setGridClass(openvdb::GRID_LEVEL_SET);
  _vertex->setTransform(_resolution);
}

voxel_pixel::voxel_pixel(voxel_pixel *grid)
    : _resolution(grid->_resolution), _materials(grid->_materials) {
  _vertex = grid->_vertex->deepCopy();
  _pixel = grid->_pixel->deepCopy();
}

bool voxel_pixel::ray_test(const vdb::math::Ray<double> &ray,
                           openvdb::Vec3d &p) {
  if (_vertex->empty())
    return false;

  openvdb::tools::LevelSetRayIntersector<vertex_grid> vray(*_vertex);

  return vray.intersectsWS(ray, p);
}

class ray_tracer_op {
public:
  ray_tracer_op(vertex_grid::Ptr grid,
                const std::vector<vdb::math::Ray<double>> &rays,
                std::vector<openvdb::Vec3d> &ps)
      : _rays(rays), _ps(ps), _vray(*grid) {}

  void operator()(const tbb::blocked_range<size_t> &range) const {
    for (size_t j = range.begin(), je = range.end(); j < je; ++j) {
      _vray.intersectsWS(_rays[j], _ps[j]);
    }
  }

private:
  const std::vector<vdb::math::Ray<double>> &_rays;
  std::vector<openvdb::Vec3d> &_ps;
  openvdb::tools::LevelSetRayIntersector<vertex_grid> _vray;
};

bool voxel_pixel::ray_test(const std::vector<vdb::math::Ray<double>> &rays,
                           std::vector<openvdb::Vec3d> &ps) {
  if (_vertex->empty())
    return false;
  ray_tracer_op op(_vertex, rays, ps);
  tbb::blocked_range<size_t> range(0, rays.size());
  tbb::parallel_for(range, op);
  return true;
}

void voxel_pixel::clear_unuse_materials() {
  std::map<std::string, bool> mapping;
  for (auto m = _materials->beginMeta(); m != _materials->endMeta(); m++) {
    mapping.emplace(m->first, true);
  }

  auto iter = get_pixel_grid()->tree().beginValueOn();
  while (iter) {
    mapping[std::to_string(iter.getValue()._data._material_id)] = false;
    ++iter;
  }

  for (auto pt : mapping) {
    if (pt.second == false)
      _materials->removeMeta(pt.first);
  }
}

void voxel_pixel::clear_unuse_features() {
  std::map<std::string, bool> mapping;
  for (auto m = _features->beginMeta(); m != _features->endMeta(); m++) {
    mapping.emplace(m->first, true);
  }

  auto iter = get_pixel_grid()->tree().beginValueOn();
  while (iter) {
    mapping[std::to_string(iter.getValue()._data._feature_id)] = false;
    ++iter;
  }

  for (auto pt : mapping) {
    if (pt.second == false)
      _features->removeMeta(pt.first);
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

int64_t voxel_pixel::get_memory_size() const {
  return _vertex->memUsage() + _pixel->memUsage();
}

bool voxel_pixel::write(const std::string &file) {
  std::ofstream os;

  os.open(file);

  if (os.is_open()) {
    _resolution->baseMap()->write(os);

    _vertex->writeTopology(os);
    _vertex->writeBuffers(os);

    _pixel->writeTopology(os);
    _pixel->writeBuffers(os);

    _materials->writeMeta(os);
    _features->writeMeta(os);

    os.close();
    return true;
  }

  os.close();
  return false;
}

bool voxel_pixel::read(const std::string &file) {
  std::ifstream is;

  is.open(file);

  if (is.is_open()) {
    _resolution->baseMap()->read(is);

    _vertex->readTopology(is);
    _vertex->readBuffers(is);

    _pixel->readTopology(is);
    _pixel->readBuffers(is);

    _materials->readMeta(is);
    _features->readMeta(is);

    _vertex->setTransform(_resolution);
    _vertex->setGridClass(vdb::GRID_LEVEL_SET);
    is.close();
    return true;
  }

  is.close();
  return false;
}

pixel_grid::Ptr voxel_pixel::extract_color(voxel_pixel &spot) {
  auto grid = vdb::tools::clip(*get_voxel_grid(), *spot.get_voxel_grid(), true);
  auto _extracted_color = pixel_grid::create();

  auto assessor = _extracted_color->getAccessor();
  auto color_accessor = get_voxel_grid()->getAccessor();
  for (auto iter = grid->tree().beginValueOn(); iter; ++iter) {
    vdb::Coord flat_coord = iter.getCoord();
    flat_coord.x() = 0;
    assessor.setValue(flat_coord, color_accessor.getValue(iter.getCoord()));
  }

  composite(spot, voxel_pixel::composite_type::op_difference);

  return _extracted_color;
}

void voxel_pixel::fill_color(voxel_pixel &spot, pixel_grid::Ptr _colors) {
  auto accessor = _colors->getAccessor();
  for (auto iter = spot.get_pixel_grid()->beginValueOn(); iter; ++iter) {
    auto coord = iter.getCoord();
    coord.x() = 0;
    if (accessor.isValueOn(coord))
      iter.setValue(accessor.getValue(coord));
  }

  composite(spot, voxel_pixel::composite_type::op_union);
}

class search_surface_op {
public:
  using index_tree = vdb::tree::Tree4<int32_t, 5, 4, 3>::Type;

public:
  search_surface_op(const bbox2<double> &range, std::vector<vdb::Coord> &nodes,
                    vertex_grid &grid, index_tree &index)
      : _nodes(nodes), _index_tree(&index), _grid(grid), _range(range) {}

  search_surface_op(search_surface_op &other, tbb::split)
      : _nodes(other._nodes), _index_tree(&_local_tree), _grid(other._grid),
        _range(other._range) {}

  void operator()(const tbb::blocked_range<size_t> &range) {
    vdb::tree::ValueAccessor<index_tree> paccess(*_index_tree);
    auto acc = _grid.getAccessor();
    for (size_t n = range.begin(), N = range.end(); n < N; ++n) {
      auto &coord = _nodes[n];

      if (!acc.isValueOn(coord) || acc.getValue(coord) > 0)
        continue;
      vdb::Coord flat_coord(0, coord.y(), coord.z());
      if (paccess.isValueOn(flat_coord)) {
        if (paccess.getValue(flat_coord) < coord.x()) {
          paccess.setValue(flat_coord, coord.x());
        }
      } else {
        paccess.setValue(flat_coord, coord.x());
      }
    }
  }

  void join(search_surface_op &surface) {
    _index_tree->merge(*surface._index_tree);
  }

private:
  std::vector<vdb::Coord> &_nodes;
  index_tree _local_tree;
  index_tree *_index_tree;
  vertex_grid &_grid;
  bbox2<double> _range;
};

vdb::BBoxd voxel_pixel::eval_max_min_elevation(vdb::BBoxd _in) {
  if (get_voxel_grid()->empty()) {
    return vdb::BBoxd({0, 0, 0}, {0, 0, 0});
  }

  std::vector<vdb::Coord> _nodes;

  vdb::CoordBBox leaf_box;
  get_voxel_grid()->tree().evalLeafBoundingBox(leaf_box);
  auto min = voxel_resolution()->worldToIndex(_in.min());
  auto max = voxel_resolution()->worldToIndex(_in.max());
  int start_x = leaf_box.min().x();
  int start_y = min.y();
  int start_z = min.z();

  int end_x = leaf_box.max().x();
  int end_y = max.y();
  int end_z = max.z();

  vdb::Coord start_coord(start_x, start_y, start_z);
  for (int x = start_x; x <= end_x; x++) {
    for (int y = start_y; y <= end_y; y++) {
      for (int z = start_z; z <= end_z; z++) {
        _nodes.emplace_back(vdb::Coord(x, y, z));
      }
    }
  }

  const tbb::blocked_range<size_t> nodeRange(0, _nodes.size());

  search_surface_op::index_tree _index_tree;
  search_surface_op op(bbox2<double>({min.y(), min.z()}, {max.y(), max.z()}),
                       _nodes, *get_voxel_grid(), _index_tree);
  // op(nodeRange);
  tbb::parallel_reduce(nodeRange, op);

  int32_t _min = std::numeric_limits<int32_t>::max();
  int32_t _max = std::numeric_limits<int32_t>::min();

  for (auto iter = _index_tree.beginValueOn(); iter; ++iter) {
    _min = std::min(_min, iter.getValue());
    _max = std::max(_max, iter.getValue());
  }

  return vdb::BBoxd(voxel_resolution()->indexToWorld(
                        vdb::Vec3d(_min, _in.min().y(), _in.min().z())),
                    voxel_resolution()->indexToWorld(
                        vdb::Vec3d(_max, _in.max().y(), _in.max().z())));
}

std::vector<std::shared_ptr<material_data>> voxel_pixel::get_materials() {
  std::vector<std::shared_ptr<material_data>> results;
  results.reserve(_materials->metaCount());
  for (auto m = _materials->beginMeta(); m != _materials->endMeta(); m++) {
    auto mtl = dynamic_cast<openvdb::MaterialMetadata *>(m->second.get());
    results.emplace_back(std::make_shared<material_data>(mtl->value()));
  }
  return results;
}

void voxel_pixel::set_materials(
    std::vector<std::shared_ptr<material_data>> mtls) {
  _materials->clearMetadata();
  for (auto m : mtls) {
    _materials->insertMeta(std::to_string(m->_material_id),
                           openvdb::MaterialMetadata(*m));
  }
}

size_t voxel_pixel::materials_count() const { return _materials->metaCount(); }

void voxel_pixel::clear_materials() const { _materials->clearMetadata(); }

void voxel_pixel::remove_material(material_id_t id) {
  if (has_material(id)) {
    _materials->removeMeta(std::to_string(id));
  }
}

bool voxel_pixel::has_material(material_id_t id) const {
  return (*_materials)[std::to_string(id)] != nullptr;
}

void voxel_pixel::add_material(std::shared_ptr<material_data> mtl) {
  auto id = mtl->_material_id;
  if (has_material(id)) {
    _materials->removeMeta(std::to_string(id));
  }
  _materials->insertMeta(std::to_string(id), openvdb::MaterialMetadata(*mtl));
}

std::shared_ptr<material_data> voxel_pixel::get_material(material_id_t id) {
  auto mtl = dynamic_cast<openvdb::MaterialMetadata *>(
      (*_materials)[std::to_string(id)].get());
  if (mtl != nullptr)
    return std::make_shared<material_data>(mtl->value());
  return nullptr;
}

std::vector<std::shared_ptr<feature_data>> voxel_pixel::get_features() {
  std::vector<std::shared_ptr<feature_data>> results;
  results.reserve(_features->metaCount());
  for (auto m = _features->beginMeta(); m != _features->endMeta(); m++) {
    auto feat = dynamic_cast<openvdb::FeatureMetadata *>(m->second.get());
    results.emplace_back(std::make_shared<feature_data>(feat->value()));
  }
  return results;
}

void voxel_pixel::set_features(
    std::vector<std::shared_ptr<feature_data>> feats) {
  _features->clearMetadata();
  for (auto m : feats) {
    _features->insertMeta(std::to_string(m->_feature_id),
                          openvdb::FeatureMetadata(*m));
  }
}

std::shared_ptr<feature_data> voxel_pixel::get_feature(local_feature_id_t id) {
  auto mtl = dynamic_cast<openvdb::FeatureMetadata *>(
      (*_features)[std::to_string(id)].get());
  if (mtl != nullptr)
    return std::make_shared<feature_data>(mtl->value());
  return nullptr;
}

size_t voxel_pixel::features_count() const { return _features->metaCount(); }

void voxel_pixel::clear_features() const { _features->clearMetadata(); }

void voxel_pixel::add_features(std::shared_ptr<feature_data> feat) {
  auto id = feat->_local_feature_id;
  if (has_material(id)) {
    _features->removeMeta(std::to_string(id));
  }
  _features->insertMeta(std::to_string(id), openvdb::FeatureMetadata(*feat));
}

void voxel_pixel::remove_feature(local_feature_id_t id) {
  if (has_feature(id)) {
    _features->removeMeta(std::to_string(id));
  }
}

bool voxel_pixel::has_feature(local_feature_id_t id) const {
  return (*_features)[std::to_string(id)] != nullptr;
}

void voxel_pixel::clear() {
  clear_materials();
  clear_features();
  _pixel->clear();
  _vertex->clear();
  _resolution = vdb::math::Transform::createLinearTransform();
}

} // namespace flywave
