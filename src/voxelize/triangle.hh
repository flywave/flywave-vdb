#pragma once

#include "bbox.hh"
#include "plan.hh"

#include <array>

#include <openvdb/Types.h>

namespace flywave {
namespace voxelize {

namespace vdb = openvdb::v8_1;

template <typename scalar_type> class triangle2 {
public:
  typedef triangle2<scalar_type> triangle2_type;
  typedef scalar_type value_type;
  typedef std::decay_t<scalar_type> value_type_t;
  typedef vdb::math::Vec2<scalar_type> vector_type;
  typedef vdb::math::Vec3<scalar_type> vector3_type;

  std::array<vector_type, 3> data_array;

  triangle2() noexcept : data_array(0) {}

  triangle2(const triangle2 &t) noexcept : data_array(t.data_array) {}

  triangle2(const vector_type &p0_, const vector_type &p1_,
            const vector_type &p2_) noexcept
      : data_array(p0_, p1_, p2_) {}

  triangle2(const vector3_type &p0_, const vector3_type &p1_,
            const vector3_type &p2_) noexcept
      : data_array(vector_type(p0_.x(), p0_.y()), vector_type(p1_.x(), p1_.y()),
                   vector_type(p2_.x(), p2_.y())) {}

  triangle2(const triangle2<scalar_type> *q2) noexcept
      : data_array(q2->data_array) {}

  triangle2(triangle2 &&q2) noexcept : data_array(std::move(q2.data_array)) {}

  triangle2 &operator=(const triangle2 &q2) noexcept {
    data_array = q2.data_array;
    return *this;
  }

  const vector_type *data() const noexcept { return data_array.data(); }

  scalar_type area() const noexcept {
    vector_type u = data_array[1] - data_array[0];
    vector_type v = data_array[2] - data_array[0];
    return scalar_type(0.5f * std::abs(u[0] * v[1] - u[1] * v[0]));
  }

  bool inside(scalar_type x, scalar_type y) const {
    scalar_type t0 = data_array[0].x() - data_array[2].x();
    scalar_type t1 = data_array[1].x() - data_array[2].x();
    scalar_type t2 = data_array[0].y() - data_array[2].y();
    scalar_type t3 = data_array[1].y() - data_array[2].y();

    scalar_type detT = t0 * t3 - t2 * t1;

    scalar_type const dx = (x - data_array[2].x());
    scalar_type const dy = (y - data_array[2].y());

    float const alpha = ((data_array[1].y() - data_array[2].y()) * dx +
                         (data_array[2].x() - data_array[1].x()) * dy) /
                        detT;
    if (alpha < 0.0f || alpha > 1.0f)
      return false;

    float const beta = ((data_array[2].y() - data_array[0].y()) * dx +
                        (data_array[0].x() - data_array[2].x()) * dy) /
                       detT;
    if (beta < 0.0f || beta > 1.0f)
      return false;

    if (alpha + beta > 1.0f)
      return false;

    return true;
  }

  vector3_type barycentric_from_coords(const vector_type &p) const {
    scalar_type t0 = data_array[0].x() - data_array[2].x();
    scalar_type t1 = data_array[1].x() - data_array[2].x();
    scalar_type t2 = data_array[0].y() - data_array[2].y();
    scalar_type t3 = data_array[1].y() - data_array[2].y();

    scalar_type detT = t0 * t3 - t2 * t1;

    scalar_type const alpha =
        ((data_array[1].y() - data_array[2].y()) * (p.x() - data_array[2].x()) +
         (data_array[2].x() - data_array[1].x()) *
             (p.y() - data_array[2].y())) /
        detT;
    scalar_type const beta =
        ((data_array[2].y() - data_array[0].y()) * (p.x() - data_array[2].x()) +
         (data_array[0].x() - data_array[2].x()) *
             (p.y() - data_array[2].y())) /
        detT;
    scalar_type const gamma = 1.0f - alpha - beta;
    return vector3_type(alpha, beta, gamma);
  }

  vector3_type barycentric_from_point(const vector3_type &p) {
    vector_type u = data_array[1] - data_array[0];
    vector_type v = data_array[2] - data_array[0];
    vector_type p_tri = p - data_array[0];
    scalar_type d00 = u.dot(u);
    scalar_type d01 = u.dot(v);
    scalar_type d11 = v.dot(v);
    scalar_type d20 = p_tri.dot(u);
    scalar_type d21 = p_tri.dot(v);

    vector3_type b;
    b[1] = (d11 * d20 - d01 * d21) / (d00 * d11 - d01 * d01);
    b[2] = (d00 * d21 - d01 * d20) / (d00 * d11 - d01 * d01);
    b[0] = scalar_type{1} - b[1] - b[2];
    return b;
  }

  vector_type point_from_barycentric(const vector3_type &barycentric_coords) {
    return data_array[0] * barycentric_coords[0] +
           data_array[1] * barycentric_coords[1] +
           data_array[2] * barycentric_coords[2];
  }

  size_t size() const noexcept { return data_array.size(); }

  vector_type &operator()(int index) { return data_array[index]; }

  vector_type operator()(int index) const { return data_array[index]; }

  vector_type &operator[](const size_t index) { return data_array[index]; }

  const vector_type &operator[](const size_t index) const {
    return data_array[index];
  }

  bool eq(const triangle2 &rhs) const { return (*this == rhs); }

  bool operator==(const triangle2 &p) const {
    return p.data_array == data_array;
  }

  bool operator!=(const triangle2 &p) const {
    return p.data_array != data_array;
  }
};

template <typename scalar_type> class triangle3 {
public:
  typedef triangle3<scalar_type> triangle3_type;
  typedef scalar_type value_type;
  typedef std::decay_t<scalar_type> value_type_t;
  typedef vdb::math::Vec3<scalar_type> vector_type;

  std::array<vector_type, 3> data_array;

  triangle3() noexcept : data_array(0) {}
  triangle3(const triangle3 &t) noexcept : data_array(t.data_array) {}
  triangle3(const vector_type &p0_, const vector_type &p1_,
            const vector_type &p2_) noexcept
      : data_array(p0_, p1_, p2_) {}

  triangle3(const triangle3<scalar_type> *q2) noexcept
      : data_array(q2->data_array) {}

  triangle3(triangle3 &&q2) noexcept : data_array(std::move(q2.data_array)) {}

  triangle3 &operator=(const triangle3 &q2) noexcept {
    data_array = q2.data_array;
    return *this;
  }

  const vector_type *data() const noexcept { return data_array.data(); }

  vector_type normal() const noexcept {
    return ((data_array[1] - data_array[0])
                .cross(data_array[2] - data_array[0]))
        .unitSafe();
  }

  plan<scalar_type> plane() const noexcept {
    auto n =
        (data_array[1] - data_array[0]).cross(data_array[2] - data_array[0]);
    n.unitSafe();
    return plan<scalar_type>(n, data_array[0]);
  }

  vector_type barycentric_from_point(const vector_type &p) {
    vector_type u = data_array[1] - data_array[0];
    vector_type v = data_array[2] - data_array[0];
    vector_type p_tri = p - data_array[0];
    scalar_type d00 = u.dot(u);
    scalar_type d01 = u.dot(v);
    scalar_type d11 = v.dot(v);
    scalar_type d20 = p_tri.dot(u);
    scalar_type d21 = p_tri.dot(v);

    vector_type b;
    b[1] = (d11 * d20 - d01 * d21) / (d00 * d11 - d01 * d01);
    b[2] = (d00 * d21 - d01 * d20) / (d00 * d11 - d01 * d01);
    b[0] = scalar_type{1} - b[1] - b[2];
    return b;
  }

  vector_type point_from_barycentric(const vector_type &barycentric_coords) {
    return data_array[0] * barycentric_coords.x() +
           data_array[1] * barycentric_coords.y() +
           data_array[2] * barycentric_coords.z();
  }

  size_t size() const noexcept { return data_array.size(); }

  vector_type &operator()(int index) { return data_array[index]; }

  vector_type operator()(int index) const { return data_array[index]; }

  vector_type &operator[](const size_t index) { return data_array[index]; }

  const vector_type &operator[](const size_t index) const {
    return data_array[index];
  }

  bool eq(const triangle3 &rhs) const { return (*this == rhs); }

  bool operator==(const triangle3 &p) const {
    return p.data_array == data_array;
  }

  bool operator!=(const triangle3 &p) const {
    return p.data_array != data_array;
  }
};

} // namespace voxelize
} // namespace flywave
