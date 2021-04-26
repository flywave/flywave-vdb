#pragma once

#include <openvdb/Types.h>

namespace flywave {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

template <typename scalar_type> class ray2 {
public:
  typedef ray2<scalar_type> ray_type;
  typedef scalar_type value_type;
  typedef std::decay_t<scalar_type> value_type_t;
  typedef vdb::math::Vec2<scalar_type> vector_type;

  vector_type origin;
  vector_type direction;

  ray2() noexcept : origin(), direction() {}
  ray2(const ray2 &r) noexcept : origin(r.origin), direction(r.direction) {}
  ray2(const vector_type &rorigin, const vector_type &rdirection) noexcept
      : origin(rorigin), direction(rdirection) {}

  vector_type get_point(const scalar_type &distance) const {
    return origin + distance * direction;
  }

  bool eq(const ray2 &rhs) const {
    return rhs.origin.eq(origin) && rhs.direction.eq(direction);
  }

  bool operator==(const ray2 &rhs) const {
    return rhs.origin.eq(origin) && rhs.direction.eq(direction);
  }

  bool operator!=(const ray2 &rhs) const { return !(*this == rhs); }
};
} // namespace flywave
