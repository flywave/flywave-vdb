#pragma once

#include <openvdb/openvdb.h>

#include <vector>

namespace flywave {

class vdb_particle {
protected:
  struct particle {
    openvdb::Vec3R p, v;
    openvdb::Real r;
  };
  openvdb::Real _radius_scale;
  openvdb::Real _velocity_scale;
  std::vector<particle> _particle_list;

public:
  typedef openvdb::Vec3R PosType;

  vdb_particle(openvdb::Real rScale = 1, openvdb::Real vScale = 1)
      : _radius_scale(rScale), _velocity_scale(vScale) {}

  void add(const openvdb::Vec3R &p, const openvdb::Real &r,
           const openvdb::Vec3R &v = openvdb::Vec3R(0, 0, 0)) {
    particle pa;
    pa.p = p;
    pa.r = r;
    pa.v = v;
    _particle_list.push_back(pa);
  }

  bool is_valid() { return (_particle_list.size() > 0) ? true : false; }

  void remove() { _particle_list.pop_back(); }

  void clear() { _particle_list.clear(); }

  openvdb::CoordBBox getBBox(const openvdb::GridBase &grid) {
    openvdb::CoordBBox bbox;
    openvdb::Coord &min = bbox.min(), &max = bbox.max();
    openvdb::Vec3R pos;
    openvdb::Real rad, invDx = 1 / grid.voxelSize()[0];
    for (size_t n = 0, e = this->size(); n < e; ++n) {
      this->getPosRad(n, pos, rad);
      const openvdb::Vec3d xyz = grid.worldToIndex(pos);
      const openvdb::Real r = rad * invDx;
      for (int i = 0; i < 3; ++i) {
        min[i] = openvdb::math::Min(min[i], openvdb::math::Floor(xyz[i] - r));
        max[i] = openvdb::math::Max(max[i], openvdb::math::Ceil(xyz[i] + r));
      }
    }
    return bbox;
  }

  openvdb::Vec3R pos(int n) const { return _particle_list[n].p; }
  openvdb::Vec3R vel(int n) const {
    return _velocity_scale * _particle_list[n].v;
  }
  openvdb::Real radius(int n) const {
    return _radius_scale * _particle_list[n].r;
  }

  size_t size() const { return _particle_list.size(); }

  void getPos(size_t n, openvdb::Vec3R &pos) const {
    pos = _particle_list[n].p;
  }

  void getPosRad(size_t n, openvdb::Vec3R &pos, openvdb::Real &rad) const {
    pos = _particle_list[n].p;
    rad = _radius_scale * _particle_list[n].r;
  }
  void getPosRadVel(size_t n, openvdb::Vec3R &pos, openvdb::Real &rad,
                    openvdb::Vec3R &vel) const {
    pos = _particle_list[n].p;
    rad = _radius_scale * _particle_list[n].r;
    vel = _velocity_scale * _particle_list[n].v;
  }

  void getAtt(size_t n, openvdb::Index32 &att) const {
    att = openvdb::Index32(n);
  }
};
} // namespace flywave
