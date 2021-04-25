#pragma once

#include <Eigen/Geometry>

namespace flywave {

template <typename scalar_type> class bbox2 {
public:
  typedef scalar_type value_type;
  typedef std::decay_t<scalar_type> value_type_t;
  typedef Eigen::Matrix<scalar_type, 2, 1> vector_type;

  static const size_t SIZE = 4;

  Eigen::Matrix<scalar_type, 2, 1> min{std::numeric_limits<scalar_type>::max()};
  Eigen::Matrix<scalar_type, 2, 1> max{
      -std::numeric_limits<scalar_type>::max()};

  bbox2() noexcept = default;
  bbox2(const bbox2 &box) noexcept : min(box.min), max(box.max) {}
  bbox2(const Eigen::Matrix<scalar_type, 2, 1> &v) noexcept : min(v), max(v) {}
  bbox2(const Eigen::Matrix<scalar_type, 2, 1> &bmin,
        const Eigen::Matrix<scalar_type, 2, 1> &bmax) noexcept
      : min(bmin), max(bmax) {}
  bbox2(const Eigen::Matrix<scalar_type, 2, 1> &bmin,
        const Eigen::Matrix<scalar_type, 2, 1> &bmax, bool sorted) noexcept
      : min(bmin), max(bmax) {
    if (!sorted)
      this->sort();
  }

  bbox2(const Eigen::Matrix<scalar_type, 2, 1> &bmin,
        scalar_type length) noexcept
      : min(bmin),
        max(bmin +
            (std::is_integral<scalar_type>::value ? length - 1 : length)) {}

  bbox2(const scalar_type *xy, bool sorted = true) noexcept
      : min({xy[0], xy[1]}), max({xy[2], xy[3]}) {
    if (!sorted)
      this->sort();
  }

  void expand(scalar_type dx) {
    dx = std::abs(dx);
#pragma unroll
    for (int i = 0; i < 2; ++i) {
      min[i] -= dx;
      max[i] += dx;
    }
  }

  void extend(const Eigen::Matrix<scalar_type, 2, 1> &v) {
    min.min(v);
    max.max(v);
  }

  void extend(const bbox2 &box) {
    min.min(box.min);
    max.max(box.max);
  }

  void expand(const Eigen::Matrix<scalar_type, 2, 1> &xyMin,
              const scalar_type &length) {
    const scalar_type size =
        std::is_integral<scalar_type>::value ? length - 1 : length;
#pragma unroll
    for (int i = 0; i < 2; ++i) {
      min[i] = std::min(min[i], xyMin[i]);
      max[i] = std::max(max[i], xyMin[i] + size);
    }
  }

  void scale(scalar_type x) noexcept {
    Eigen::Matrix<scalar_type, 2, 1> const v = x * halfsize();
    min -= v;
    max += v;
  }

  scalar_type width() const { return max.x - min.x; }

  scalar_type height() const { return max.y - min.y; }

  scalar_type area() const {
    Eigen::Matrix<scalar_type, 2, 1> d = max - min;
    return d[0] * d[1];
  }

  void translate(const Eigen::Matrix<scalar_type, 2, 1> &dx) {
    min += dx;
    max += dx;
  }

  void sort() {
    Eigen::Matrix<scalar_type, 2, 1> tMin(min), tMax(max);
#pragma unroll
    for (int i = 0; i < 2; ++i) {
      min[i] = std::min(tMin[i], tMax[i]);
      max[i] = std::max(tMin[i], tMax[i]);
    }
  }

  bool is_sorted() const {
    if (std::is_integral<scalar_type>::value) {
      return (min.x <= max.x && min.y <= max.y);
    } else {
      scalar_type t = tolerance<scalar_type>();
      return (min.x < (max.x + t) && min.y < (max.y + t));
    }
  }

  Eigen::Matrix<scalar_type, 3, 1> extents() const {
    if (std::is_integral<scalar_type>::value) {
      return (max - min) + Eigen::Matrix<scalar_type, 2, 1>(1, 1);
    } else {
      return (max - min);
    }
  }

  Eigen::Matrix<scalar_type, 2, 1> diagonal() const { return max - min; }

  Eigen::Matrix<scalar_type, 2, 1> center() const { return (min + max) * 0.5f; }

  Eigen::Matrix<scalar_type, 2, 1> center2() const { return (min + max); }

  Eigen::Matrix<scalar_type, 2, 1> halfsize() const {
    return 0.5f * (max - min);
  }

  inline scalar_type const &operator[](size_t i) const {
    if (i < 2)
      return min[i];
    else
      return max[i];
  }

  inline scalar_type &operator[](size_t i) {
    if (i < 2)
      return min[i];
    else
      return max[i];
  }

  bbox2 &operator=(const bbox2 &box) {
    min = box.min;
    max = box.max;
    return *this;
  }

  bool is_equal(const bbox2 &rhs) const { return (*this == rhs); }

  bool operator==(const bbox2 &rhs) const {
    if (std::is_integral<scalar_type>::value) {
      return min == rhs.min && max == rhs.max;
    } else {
      return math::is_approx_equal(min, rhs.min) &&
             math::is_approx_equal(max, rhs.max);
    }
  }

  bool operator!=(const bbox2 &rhs) const { return !(*this == rhs); }

  bool empty() const {
    if (std::is_integral<scalar_type>::value) {
      return (min.x > max.x || min.y > max.y);
    }
    return min.x >= max.x || min.y >= max.y;
  }

  Eigen::Matrix<scalar_type, 2, 1> intersect(const ray2<scalar_type> &r) const {
    Eigen::Matrix<scalar_type, 2, 1> v1 = min, v2 = max;
    const Eigen::Matrix<scalar_type, 2, 1> bbox_eps{0.0000001f};

    Eigen::Matrix<scalar_type, 2, 1> div = r.direction;
    div.set_if(div.abs() < bbox_eps, bbox_eps);

    Eigen::Matrix<scalar_type, 2, 1> t1 = (v1 - r.origin) / div;
    Eigen::Matrix<scalar_type, 2, 1> t2 = (v2 - r.origin) / div;

    Eigen::Matrix<scalar_type, 2, 1> tmin = t1.minned(t2);
    Eigen::Matrix<scalar_type, 2, 1> tmax = t1.maxed(t2);

    return {tmin.max_element(), tmax.min_element()};
  }

  bool contains(const Eigen::Matrix<scalar_type, 2, 1> &p) const {
    if (std::is_integral<scalar_type>::value) {
      if (((p.x >= min.x && p.x <= max.x) || (p.x <= min.x && p.x >= max.x)) &&
          ((p.y >= min.y && p.y <= max.y) || (p.y <= min.y && p.y >= max.y))) {
        return true;
      }
      return false;
    } else {
      scalar_type t = tolerance<scalar_type>();
      if (((p.x >= (min.x - t) && p.x <= (max.x - t)) ||
           (p.x <= (min.x - t) && p.x >= (max.x - t)) &&
               ((p.y >= (min.y - t) && p.y <= (max.y - t)) ||
                (p.y <= (min.y - t) && p.y >= (max.y - t))))) {
        return true;
      }
      return false;
    }
  }

  bool contains(const bbox2 &b) const {
    if (std::is_integral<scalar_type>::value) {
      return b.min.x >= min.x && b.max.x <= max.x && b.min.y >= min.y &&
             b.max.y <= max.y;
    } else {
      scalar_type t = tolerance<scalar_type>();
      return (b.min.x - t) > min.x && (b.max.x + t) < max.x &&
             (b.min.y - t) > min.y && (b.max.y + t) < max.y;
    }
  }

  bool overlap(const bbox2 &b) const {
    if (std::is_integral<scalar_type>::value) {
      return max.x >= b.min.x && min.x <= b.max.x && max.y >= b.min.y &&
             min.y <= b.max.y;
    } else {
      scalar_type t = tolerance<scalar_type>();
      return max.x > (b.min.x - t) && min.x < (b.max.x + t) &&
             max.y > (b.min.y - t) && min.y < (b.max.y + t);
    }
  }
};

template <typename T> bbox2<T> operator*(const float &a, const bbox2<T> &b) {
  return bbox2<T>(a * b.min, a * b.max);
}

template <typename T>
bbox2<T> operator*(const Eigen::Matrix<T, 2, 1> &a, const bbox2<T> &b) {
  return bbox2<T>(a * b.min, a * b.max);
}

template <typename T> bbox2<T> operator+(const bbox2<T> &a, const bbox2<T> &b) {
  return bbox2<T>(a.min + b.min, a.max + b.max);
}

template <typename T> bbox2<T> operator-(const bbox2<T> &a, const bbox2<T> &b) {
  return bbox2<T>(a.min - b.min, a.max - b.max);
}

template <typename T>
bbox2<T> operator+(const bbox2<T> &a, const Eigen::Matrix<T, 2, 1> &b) {
  return bbox2<T>(a.min + b, a.max + b);
}

template <typename T>
bbox2<T> operator-(const bbox2<T> &a, const Eigen::Matrix<T, 2, 1> &b) {
  return bbox2<T>(a.min - b, a.max - b);
}

template <typename T>
bbox2<T> enlarge(const bbox2<T> &a, const Eigen::Matrix<T, 2, 1> &b) {
  return bbox2<T>(a.min - b, a.max + b);
}

template <typename T>
const bbox2<T> merge(const bbox2<T> &a, const Eigen::Matrix<T, 2, 1> &b) {
  return bbox2<T>(math::min(a.min, b), math::max(a.max, b));
}

template <typename T>
const bbox2<T> merge(const Eigen::Matrix<T, 2, 1> &a, const bbox2<T> &b) {
  return bbox2<T>(math::min(a, b.min), math::max(a, b.max));
}

template <typename T>
const bbox2<T> merge(const bbox2<T> &a, const bbox2<T> &b) {
  return bbox2<T>(math::min(a.min, b.min), math::max(a.max, b.max));
}

template <typename T>
const bbox2<T> merge(const bbox2<T> &a, const bbox2<T> &b, const bbox2<T> &c) {
  return merge(a, merge(b, c));
}

template <typename T>
bbox2<T> merge(const bbox2<T> &a, const bbox2<T> &b, const bbox2<T> &c,
               const bbox2<T> &d) {
  return merge(merge(a, b), merge(c, d));
}

template <typename T>
const bbox2<T> intersect(const bbox2<T> &a, const bbox2<T> &b) {
  return bbox2<T>(math::max(a.min, b.min), math::min(a.max, b.max));
}

template <typename T>
const bbox2<T> intersect(const bbox2<T> &a, const bbox2<T> &b,
                         const bbox2<T> &c) {
  return intersect(a, intersect(b, c));
}

template <typename T>
const bbox2<T> intersect(const bbox2<T> &a, const bbox2<T> &b,
                         const bbox2<T> &c, const bbox2<T> &d) {
  return intersect(intersect(a, b), intersect(c, d));
}

template <typename T> bool disjoint(const bbox2<T> &a, const bbox2<T> &b) {
  return intersect(a, b).empty();
}
template <typename T>
bool disjoint(const bbox2<T> &a, const Eigen::Matrix<T, 2, 1> &b) {
  return disjoint(a, bbox2<T>(b));
}
template <typename T>
bool disjoint(const Eigen::Matrix<T, 2, 1> &a, const bbox2<T> &b) {
  return disjoint(bbox2<T>(a), b);
}

template <typename T> bool conjoint(const bbox2<T> &a, const bbox2<T> &b) {
  return !intersect(a, b).empty();
}
template <typename T>
bool conjoint(const bbox2<T> &a, const Eigen::Matrix<T, 2, 1> &b) {
  return conjoint(a, bbox2<T>(b));
}
template <typename T>
bool conjoint(const Eigen::Matrix<T, 2, 1> &a, const bbox2<T> &b) {
  return conjoint(bbox2<T>(a), b);
}

template <typename T>
void subtract(const bbox2<T> &a, const bbox2<T> &b, bbox2<T> &c, bbox2<T> &d) {
  c.min = a.min;
  c.max = math::min(a.max, b.min);
  d.min = math::max(a.min, b.max);
  d.max = a.max;
}

template <typename T> bool subset(const bbox2<T> &a, const bbox2<T> &b) {
  for (size_t i = 0; i < T::N; i++)
    if (a.min[i] < b.min[i])
      return false;
  for (size_t i = 0; i < T::N; i++)
    if (a.max[i] > b.max[i])
      return false;
  return true;
}

template <typename scalar_type> class bbox3 {
public:
  typedef scalar_type value_type;
  typedef std::decay_t<scalar_type> value_type_t;
  typedef Eigen::Matrix<scalar_type, 3, 1> vector_type;

  static const size_t SIZE = 6;

  Eigen::Matrix<scalar_type, 3, 1> min{std::numeric_limits<scalar_type>::max()};
  Eigen::Matrix<scalar_type, 3, 1> max{
      -std::numeric_limits<scalar_type>::max()};

  bbox3() noexcept = default;
  bbox3(const bbox3 &box) noexcept : min(box.min), max(box.max) {}
  bbox3(const Eigen::Matrix<scalar_type, 3, 1> &v) noexcept : min(v), max(v) {}
  bbox3(const Eigen::Matrix<scalar_type, 3, 1> &bmin,
        const Eigen::Matrix<scalar_type, 3, 1> &bmax) noexcept
      : min(bmin), max(bmax) {}
  bbox3(const Eigen::Matrix<scalar_type, 3, 1> &bmin,
        const Eigen::Matrix<scalar_type, 3, 1> &bmax, bool sorted) noexcept
      : min(bmin), max(bmax) {
    if (!sorted)
      this->sort();
  }

  bbox3(const Eigen::Matrix<scalar_type, 3, 1> &bmin,
        scalar_type length) noexcept
      : min(bmin),
        max(bmin +
            (std::is_integral<scalar_type>::value ? length - 1 : length)) {}

  bbox3(const scalar_type *xyz, bool sorted = true) noexcept
      : min({xyz[0], xyz[1], xyz[2]}), max({xyz[3], xyz[4], xyz[5]}) {
    if (!sorted)
      this->sort();
  }

  void expand(scalar_type dx) {
    dx = std::abs(dx);
#pragma unroll
    for (int i = 0; i < 3; ++i) {
      min[i] -= dx;
      max[i] += dx;
    }
  }

  void extend(const Eigen::Matrix<scalar_type, 3, 1> &v) {
    min.min(v);
    max.max(v);
  }

  void extend(const bbox3 &box) {
    min.min(box.min);
    max.max(box.max);
  }

  void expand(const Eigen::Matrix<scalar_type, 3, 1> &xyzMin,
              const scalar_type &length) {
    const scalar_type size =
        std::is_integral<scalar_type>::value ? length - 1 : length;
#pragma unroll
    for (int i = 0; i < 3; ++i) {
      min[i] = std::min(min[i], xyzMin[i]);
      max[i] = std::max(max[i], xyzMin[i] + size);
    }
  }

  void scale(scalar_type x) noexcept {
    Eigen::Matrix<scalar_type, 3, 1> const v = x * halfsize();
    min -= v;
    max += v;
  }

  void translate(const Eigen::Matrix<scalar_type, 3, 1> &dx) {
    min += dx;
    max += dx;
  }

  void sort() {
    Eigen::Matrix<scalar_type, 3, 1> tMin(min), tMax(max);
#pragma unroll
    for (int i = 0; i < 3; ++i) {
      min[i] = std::min(tMin[i], tMax[i]);
      max[i] = std::max(tMin[i], tMax[i]);
    }
  }

  bool is_sorted() const {
    if (std::is_integral<scalar_type>::value) {
      return (min.x <= max.x && min.y <= max.y && min.z <= max.z);
    } else {
      scalar_type t = tolerance<scalar_type>();
      return (min.x < (max.x + t) && min.y < (max.y + t) &&
              min.z < (max.z + t));
    }
  }

  bool empty() const {
    if (std::is_integral<scalar_type>::value) {
      return (min.x > max.x || min.y > max.y || min.z > max.z);
    }
    return min.x >= max.x || min.y >= max.y || min.z >= max.z;
  }

  Eigen::Matrix<scalar_type, 3, 1> extents() const {
    if (std::is_integral<scalar_type>::value) {
      return (max - min) + Eigen::Matrix<scalar_type, 3, 1>(1, 1, 1);
    } else {
      return (max - min);
    }
  }

  scalar_type width() const { return max.x - min.x; }

  scalar_type height() const { return max.y - min.y; }

  scalar_type depth() const { return max.z - min.z; }

  scalar_type surface_area() const {
    Eigen::Matrix<scalar_type, 3, 1> d = max - min;
    return 2.f * (d[0] * d[1] + d[0] * d[2] + d[1] * d[2]);
  }

  scalar_type volume() const {
    Eigen::Matrix<scalar_type, 3, 1> d = max - min;
    return d[0] * d[1] * d[2];
  }

  Eigen::Matrix<scalar_type, 3, 1> diagonal() const { return max - min; }

  Eigen::Matrix<scalar_type, 3, 1> center() const { return (min + max) * 0.5f; }

  Eigen::Matrix<scalar_type, 3, 1> center2() const { return (min + max); }

  Eigen::Matrix<scalar_type, 3, 1> halfsize() const {
    return 0.5f * (max - min);
  }

  size_t max_extent() const { return math::max_index(max - min); }

  size_t min_extent() const { return math::min_index(max - min); }

  inline scalar_type const &operator[](size_t i) const {
    if (i < 3)
      return min[i];
    else
      return max[i];
  }

  inline scalar_type &operator[](size_t i) {
    if (i < 3)
      return min[i];
    else
      return max[i];
  }

  bbox3 &operator=(const bbox3 &box) {
    min = box.min;
    max = box.max;
    return *this;
  }

  bool is_equal(const bbox3 &rhs) const { return (*this == rhs); }

  bool operator==(const bbox3 &rhs) const {
    if (std::is_integral<scalar_type>::value) {
      return min.is_equal(rhs.min) && max.is_equal(rhs.max);
    } else {
      return math::is_approx_equal(min, rhs.min) &&
             math::is_approx_equal(max, rhs.max);
    }
  }

  bool operator!=(const bbox3 &rhs) const { return !(*this == rhs); }

  Eigen::Matrix<scalar_type, 2, 1> intersect(const ray3<scalar_type> &r) const {
    Eigen::Matrix<scalar_type, 3, 1> v1 = min, v2 = max;
    const Eigen::Matrix<scalar_type, 3, 1> bbox_eps{0.0000001f};

    Eigen::Matrix<scalar_type, 3, 1> div = r.direction;
    div.set_if(div.abs() < bbox_eps, bbox_eps);

    Eigen::Matrix<scalar_type, 3, 1> t1 = (v1 - r.origin) / div;
    Eigen::Matrix<scalar_type, 3, 1> t2 = (v2 - r.origin) / div;

    Eigen::Matrix<scalar_type, 3, 1> tmin = t1.minned(t2);
    Eigen::Matrix<scalar_type, 3, 1> tmax = t1.maxed(t2);

    return {tmin.max_element(), tmax.min_element()};
  }

  bool contains(const Eigen::Matrix<scalar_type, 3, 1> &p) const {
    if (std::is_integral<scalar_type>::value) {
      if (((p.x >= min.x && p.x <= max.x) || (p.x <= min.x && p.x >= max.x)) &&
          ((p.y >= min.y && p.y <= max.y) || (p.y <= min.y && p.y >= max.y)) &&
          ((p.z >= min.z && p.z <= max.z) || (p.z <= min.z && p.z >= max.z))) {
        return true;
      }
      return false;
    } else {
      scalar_type t = tolerance<scalar_type>();
      if (((p.x >= (min.x - t) && p.x <= (max.x - t)) ||
           (p.x <= (min.x - t) && p.x >= (max.x - t)) &&
               ((p.y >= (min.y - t) && p.y <= (max.y - t)) ||
                (p.y <= (min.y - t) && p.y >= (max.y - t))) &&
               ((p.z >= (min.z - t) && p.z <= (max.z - t)) ||
                (p.z <= (min.z - t) && p.z >= (max.z - t))))) {
        return true;
      }
      return false;
    }
  }

  bool contains(const bbox3 &b) const {
    if (std::is_integral<scalar_type>::value) {
      return b.min.x >= min.x && b.max.x <= max.x && b.min.y >= min.y &&
             b.max.y <= max.y && b.min.z >= min.z && b.max.z <= max.z;
    } else {
      scalar_type t = tolerance<scalar_type>();
      return (b.min.x - t) > min.x && (b.max.x + t) < max.x &&
             (b.min.y - t) > min.y && (b.max.y + t) < max.y &&
             (b.min.z - t) > min.z && (b.max.z + t) < max.z;
    }
  }

  bool overlap(const bbox3 &b) const {
    if (std::is_integral<scalar_type>::value) {
      return max.x >= b.min.x && min.x <= b.max.x && max.y >= b.min.y &&
             min.y <= b.max.y && max.z >= b.min.z && min.z <= b.max.z;
    } else {
      scalar_type t = tolerance<scalar_type>();
      return max.x > (b.min.x - t) && min.x < (b.max.x + t) &&
             max.y > (b.min.y - t) && min.y < (b.max.y + t) &&
             max.z > (b.min.z - t) && min.z < (b.max.z + t);
    }
  }
};

template <typename T> bbox3<T> operator*(const float &a, const bbox3<T> &b) {
  return bbox3<T>(a * b.min, a * b.max);
}

template <typename T>
bbox3<T> operator*(const Eigen::Matrix<T, 3, 1> &a, const bbox3<T> &b) {
  return bbox3<T>(a * b.min, a * b.max);
}

template <typename T> bbox3<T> operator+(const bbox3<T> &a, const bbox3<T> &b) {
  return bbox3<T>(a.min + b.min, a.max + b.max);
}

template <typename T> bbox3<T> operator-(const bbox3<T> &a, const bbox3<T> &b) {
  return bbox3<T>(a.min - b.min, a.max - b.max);
}

template <typename T>
bbox3<T> operator+(const bbox3<T> &a, const Eigen::Matrix<T, 3, 1> &b) {
  return bbox3<T>(a.min + b, a.max + b);
}

template <typename T>
bbox3<T> operator-(const bbox3<T> &a, const Eigen::Matrix<T, 3, 1> &b) {
  return bbox3<T>(a.min - b, a.max - b);
}

template <typename T>
bbox3<T> enlarge(const bbox3<T> &a, const Eigen::Matrix<T, 3, 1> &b) {
  return bbox3<T>(a.min - b, a.max + b);
}

template <typename T>
const bbox3<T> merge(const bbox3<T> &a, const Eigen::Matrix<T, 3, 1> &b) {
  return bbox3<T>(math::min(a.min, b), math::max(a.max, b));
}

template <typename T>
const bbox3<T> merge(const Eigen::Matrix<T, 3, 1> &a, const bbox3<T> &b) {
  return bbox3<T>(math::min(a, b.min), math::max(a, b.max));
}

template <typename T>
const bbox3<T> merge(const bbox3<T> &a, const bbox3<T> &b) {
  return bbox3<T>(math::min(a.min, b.min), math::max(a.max, b.max));
}

template <typename T>
const bbox3<T> merge(const bbox3<T> &a, const bbox3<T> &b, const bbox3<T> &c) {
  return merge(a, merge(b, c));
}

template <typename T>
bbox3<T> merge(const bbox3<T> &a, const bbox3<T> &b, const bbox3<T> &c,
               const bbox3<T> &d) {
  return merge(merge(a, b), merge(c, d));
}

template <typename T>
const bbox3<T> intersect(const bbox3<T> &a, const bbox3<T> &b) {
  return bbox3<T>(math::max(a.min, b.min), math::min(a.max, b.max));
}

template <typename T>
const bbox3<T> intersect(const bbox3<T> &a, const bbox3<T> &b,
                         const bbox3<T> &c) {
  return intersect(a, intersect(b, c));
}

template <typename T>
const bbox3<T> intersect(const bbox3<T> &a, const bbox3<T> &b,
                         const bbox3<T> &c, const bbox3<T> &d) {
  return intersect(intersect(a, b), intersect(c, d));
}

template <typename T> bool disjoint(const bbox3<T> &a, const bbox3<T> &b) {
  return intersect(a, b).empty();
}
template <typename T>
bool disjoint(const bbox3<T> &a, const Eigen::Matrix<T, 3, 1> &b) {
  return disjoint(a, bbox3<T>(b));
}
template <typename T>
bool disjoint(const Eigen::Matrix<T, 3, 1> &a, const bbox3<T> &b) {
  return disjoint(bbox3<T>(a), b);
}

template <typename T> bool conjoint(const bbox3<T> &a, const bbox3<T> &b) {
  return !intersect(a, b).empty();
}
template <typename T>
bool conjoint(const bbox3<T> &a, const Eigen::Matrix<T, 3, 1> &b) {
  return conjoint(a, bbox3<T>(b));
}
template <typename T>
bool conjoint(const Eigen::Matrix<T, 3, 1> &a, const bbox3<T> &b) {
  return conjoint(bbox3<T>(a), b);
}

template <typename T>
void subtract(const bbox3<T> &a, const bbox3<T> &b, bbox3<T> &c, bbox3<T> &d) {
  c.min = a.min;
  c.max = math::min(a.max, b.min);
  d.min = math::max(a.min, b.max);
  d.max = a.max;
}

template <typename T> bool subset(const bbox3<T> &a, const bbox3<T> &b) {
  for (size_t i = 0; i < T::N; i++)
    if (a.min[i] < b.min[i])
      return false;
  for (size_t i = 0; i < T::N; i++)
    if (a.max[i] > b.max[i])
      return false;
  return true;
}

} // namespace flywave