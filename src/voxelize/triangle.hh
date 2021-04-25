#pragma once

#include "bbox.hh"

#include <Eigen/Geometry>

namespace flywave {
namespace voxelize {

template <typename scalar_type> class triangle2 {
public:
  typedef triangle2<scalar_type> triangle2_type;
  typedef scalar_type value_type;
  typedef std::decay_t<scalar_type> value_type_t;
  typedef Eigen::Matrix<scalar_type, 2, 1> vector_type;

  Eigen::Matrix<scalar_type, 2, 3> data_array;

  template <typename Describer> auto describe_type(Describer f) {
    return f(data_array.col(0), data_array.col(1), data_array.col(2));
  }
  triangle2() noexcept : data_array({}) {}

  triangle2(const triangle2 &t) noexcept : data_array(t.data_array) {}

  triangle2(const Eigen::Matrix<scalar_type, 2, 1> &p0_,
            const Eigen::Matrix<scalar_type, 2, 1> &p1_,
            const Eigen::Matrix<scalar_type, 2, 1> &p2_) noexcept
      : data_array() {
    data_array << p0_, p1_, p2_;
  }

  triangle2(const triangle2<scalar_type> *q2) noexcept
      : data_array(q2->data_array) {}

  triangle2(triangle2 &&q2) noexcept : data_array(std::move(q2.data_array)) {}

  triangle2 &operator=(const triangle2 &q2) noexcept {
    data_array = q2.data_array;
    return *this;
  }

  const vector_type *data() const noexcept { return data_array.data(); }

  scalar_type area() const noexcept {
    Eigen::Matrix<scalar_type, 2, 1> u = data_array.col(1) - data_array.col(0);
    Eigen::Matrix<scalar_type, 2, 1> v = data_array.col(2) - data_array.col(0);

    return scalar_type(0.5f * std::abs(u[0] * v[1] - u[1] * v[0]));
  }

  bool inside(scalar_type x, scalar_type y) const {
    scalar_type t0 = data_array.col(0).x() - data_array.col(2).x();
    scalar_type t1 = data_array.col(1).x() - data_array.col(2).x();
    scalar_type t2 = data_array.col(0).y() - data_array.col(2).y();
    scalar_type t3 = data_array.col(1).y() - data_array.col(2).y();

    scalar_type detT = t0 * t3 - t2 * t1;

    scalar_type const dx = (x - data_array.col(2).x());
    scalar_type const dy = (y - data_array.col(2).y());

    float const alpha = ((data_array.col(1).y() - data_array.col(2).y()) * dx +
                         (data_array.col(2).x() - data_array.col(1).x()) * dy) /
                        detT;
    if (alpha < 0.0f || alpha > 1.0f)
      return false;

    float const beta = ((data_array.col(2).y() - data_array.col(0).y()) * dx +
                        (data_array.col(0).x() - data_array.col(2).x()) * dy) /
                       detT;
    if (beta < 0.0f || beta > 1.0f)
      return false;

    if (alpha + beta > 1.0f)
      return false;

    return true;
  }

  Eigen::Matrix<scalar_type, 3, 1>
  barycentric_from_coords(const Eigen::Matrix<scalar_type, 2, 1> &p) const {
    scalar_type t0 = data_array.col(0).x() - data_array.col(2).x();
    scalar_type t1 = data_array.col(1).x() - data_array.col(2).x();
    scalar_type t2 = data_array.col(0).y() - data_array.col(2).y();
    scalar_type t3 = data_array.col(1).y() - data_array.col(2).y();

    scalar_type detT = t0 * t3 - t2 * t1;

    scalar_type const alpha = ((data_array.col(1).y() - data_array.col(2).y()) *
                                   (p.x - data_array.col(2).x()) +
                               (data_array.col(2).x() - data_array.col(1).x()) *
                                   (p.y - data_array.col(2).y())) /
                              detT;
    scalar_type const beta = ((data_array.col(2).y() - data_array.col(0).y()) *
                                  (p.x - data_array.col(2).x()) +
                              (data_array.col(0).x() - data_array.col(2).x()) *
                                  (p.y - data_array.col(2).y())) /
                             detT;
    scalar_type const gamma = 1.0f - alpha - beta;
    return Eigen::Matrix<scalar_type, 3, 1>(alpha, beta, gamma);
  }

  Eigen::Matrix<scalar_type, 3, 1>
  barycentric_from_point(const Eigen::Matrix<scalar_type, 3, 1> &p) {
    Eigen::Matrix<scalar_type, 2, 1> u = data_array.col(1) - data_array.col(0);
    Eigen::Matrix<scalar_type, 2, 1> v = data_array.col(2) - data_array.col(0);
    Eigen::Matrix<scalar_type, 2, 1> p_tri = p - data_array.col(0);
    scalar_type d00 = u.dot(u);
    scalar_type d01 = u.dot(v);
    scalar_type d11 = v.dot(v);
    scalar_type d20 = p_tri.dot(u);
    scalar_type d21 = p_tri.dot(v);

    Eigen::Matrix<scalar_type, 3, 1> b;
    b[1] = (d11 * d20 - d01 * d21) / (d00 * d11 - d01 * d01);
    b[2] = (d00 * d21 - d01 * d20) / (d00 * d11 - d01 * d01);
    b[0] = scalar_type{1} - b[1] - b[2];
    return b;
  }

  Eigen::Matrix<scalar_type, 2, 1> point_from_barycentric(
      const Eigen::Matrix<scalar_type, 3, 1> &barycentric_coords) {
    return data_array.col(0) * barycentric_coords[0] +
           data_array.col(1) * barycentric_coords[1] +
           data_array.col(2) * barycentric_coords[2];
  }

  size_t size() const noexcept { return data_array.size(); }

  vector_type &operator()(int index) { return data_array.col(index); }

  vector_type operator()(int index) const { return data_array.col(index); }

  vector_type &operator[](const size_t index) { return data_array.col(index); }

  const vector_type &operator[](const size_t index) const {
    return data_array.col(index);
  }

  bool is_equal(const triangle2 &rhs) const { return (*this == rhs); }

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
  typedef Eigen::Matrix<scalar_type, 3, 1> vector_type;

  Eigen::Matrix<scalar_type, 3, 3> data_array;

  triangle3() noexcept : data_array(0) {}
  triangle3(const triangle3 &t) noexcept : data_array(t.data_array) {}
  triangle3(const Eigen::Matrix<scalar_type, 3, 1> &p0_,
            const Eigen::Matrix<scalar_type, 3, 1> &p1_,
            const Eigen::Matrix<scalar_type, 3, 1> &p2_) noexcept
      : data_array() {
    data_array << p0_, p1_, p2_;
  }

  triangle3(const triangle3<scalar_type> *q2) noexcept
      : data_array(q2->data_array) {}

  triangle3(triangle3 &&q2) noexcept : data_array(std::move(q2.data_array)) {}

  triangle3 &operator=(const triangle3 &q2) noexcept {
    data_array = q2.data_array;
    return *this;
  }

  const vector_type *data() const noexcept { return data_array.data(); }

  Eigen::Matrix<scalar_type, 3, 1> normal() const noexcept {
    return ((data_array.col(1) - data_array.col(0)) *
            (data_array.col(2) - data_array.col(0)))
        .normalized();
  }

  Eigen::Matrix<scalar_type, 3, 1>
  barycentric_from_point(const Eigen::Matrix<scalar_type, 3, 1> &p) {
    Eigen::Matrix<scalar_type, 3, 1> u = data_array.col(1) - data_array.col(0);
    Eigen::Matrix<scalar_type, 3, 1> v = data_array.col(2) - data_array.col(0);
    Eigen::Matrix<scalar_type, 3, 1> p_tri = p - data_array.col(0);
    scalar_type d00 = u.dot(u);
    scalar_type d01 = u.dot(v);
    scalar_type d11 = v.dot(v);
    scalar_type d20 = p_tri.dot(u);
    scalar_type d21 = p_tri.dot(v);

    Eigen::Matrix<scalar_type, 3, 1> b;
    b[1] = (d11 * d20 - d01 * d21) / (d00 * d11 - d01 * d01);
    b[2] = (d00 * d21 - d01 * d20) / (d00 * d11 - d01 * d01);
    b[0] = scalar_type{1} - b[1] - b[2];
    return b;
  }

  Eigen::Matrix<scalar_type, 3, 1> point_from_barycentric(
      const Eigen::Matrix<scalar_type, 3, 1> &barycentric_coords) {
    return data_array.col(0) * barycentric_coords[0] +
           data_array.col(1) * barycentric_coords[1] +
           data_array.col(2) * barycentric_coords[2];
  }

  size_t size() const noexcept { return data_array.size(); }

  vector_type &operator()(int index) { return data_array.col(index); }

  vector_type operator()(int index) const { return data_array.col(index); }

  vector_type &operator[](const size_t index) { return data_array.col(index); }

  const vector_type &operator[](const size_t index) const {
    return data_array.col(index);
  }

  bool is_equal(const triangle3 &rhs) const { return (*this == rhs); }

  bool operator==(const triangle3 &p) const {
    return p.data_array == data_array;
  }

  bool operator!=(const triangle3 &p) const {
    return p.data_array != data_array;
  }
};

} // namespace voxelize
} // namespace flywave