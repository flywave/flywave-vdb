#pragma once

#include <openvdb/math/Ray.h>

namespace flywave {

namespace vdb = openvdb::v8_1;

template <typename scalar_type> class plan {
public:
  typedef plan<scalar_type> plan_type;
  typedef scalar_type value_type;
  typedef std::decay_t<scalar_type> value_type_t;
  typedef vdb::math::Vec3<scalar_type> vector_type;
  typedef vdb::math::Vec3<value_type_t> vector_type_t;

  vdb::math::Vec3<scalar_type> normal;
  scalar_type distance;

  plan() noexcept : normal(), distance() {}
  plan(const plan &r) noexcept : normal(r.normal), distance(r.distance) {}
  plan(const vdb::math::Vec3<scalar_type> &normal,
       const scalar_type &distance) noexcept
      : normal(normal), distance(distance) {}
  plan(const vdb::math::Vec3<scalar_type> &normal,
       const vdb::math::Vec3<scalar_type> &point) noexcept
      : normal(normal), distance(-(point).dot(normal)) {}
  plan(const vdb::math::Vec3<scalar_type> &v0,
       const vdb::math::Vec3<scalar_type> &v1,
       const vdb::math::Vec3<scalar_type> &v2) noexcept
      : plan((v2 - v1).cross(v0 - v1).normalize(), v0) {}

  plan(plan &&r) noexcept : normal(r.normal), distance(r.distance) {}

  plan(const plan *r) noexcept : normal(r->normal), distance(r->distance) {}

  plan &operator=(const plan &c) {
    normal = c.normal;
    distance = c.distance;
    return *this;
  }

  bool is_equal(const plan &rhs) const { return (*this == rhs); }

  bool operator==(const plan &rhs) const {
    return rhs.distance == distance && rhs.normal.is_equal(normal);
  }

  bool operator!=(const plan &rhs) const { return !(*this == rhs); }

  template <typename Describer> auto describe_type(Describer f) {
    return f(normal, distance);
  }

  scalar_type
  distance_to_point(const vdb::math::Vec3<scalar_type> &point) const {
    return (point).dot(normal) + distance;
  }

  vector_type
  project_point(const vdb::math::Vec3<scalar_type> &point) const {
    return normal * (-distance_to_point(point)) + point;
  }

  scalar_type dot(const vdb::math::Vec3<scalar_type> &point) const {
    return normal.dot(point);
  }

  bool behind(const vdb::math::Vec3<scalar_type> &point) const {
    return dot(point) < 0.f;
  }

  vdb::math::Vec3<scalar_type> position() const {
    return normal * distance;
  }

  bool intersects(const vdb::math::Ray<scalar_type> &ray) const {
    scalar_type t0 = vdb::math::Delta<scalar_type>();
    scalar_type t1 = std::numeric_limits<scalar_type>::max();
    const scalar_type cosAngle = ray.direction.dot(normal);
    if (vdb::math::isApproxEqual(cosAngle))
      return false;
    scalar_type t = (distance - ray.origin.dot(normal)) / cosAngle;
    return t >= t0 && t <= t1;
  }
};

} // namespace flywave
