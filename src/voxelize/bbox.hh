#pragma once

#include "ray.hh"
#include "tolerance.hh"

#include <openvdb/Types.h>

namespace flywave {

namespace vdb = openvdb::v8_1;

template <typename scalar_type> class bbox2 {
public:
  typedef scalar_type value_type;
  typedef std::decay_t<scalar_type> value_type_t;
  typedef vdb::math::Vec2<scalar_type> vector_type;
  typedef vdb::math::Vec3<scalar_type> vector3_type;

  constexpr static const size_t SIZE = 4;

  vector_type min{std::numeric_limits<scalar_type>::max()};
  vector_type max{-std::numeric_limits<scalar_type>::max()};

  bbox2() = default;
  bbox2(const bbox2 &box) : min(box.min), max(box.max) {}
  bbox2(const vector_type &v) : min(v), max(v) {}
  bbox2(const vector_type &bmin, const vector_type &bmax)
      : min(bmin), max(bmax) {}
  bbox2(const vector_type &bmin, const vector_type &bmax, bool sorted)
      : min(bmin), max(bmax) {
    if (!sorted)
      this->sort();
  }

  bbox2(const vector_type &bmin, scalar_type length)
      : min(bmin),
        max(bmin +
            (std::is_integral<scalar_type>::value ? length - 1 : length)) {}

  bbox2(const scalar_type *xy, bool sorted = true)
      : min(xy[0], xy[1]), max(xy[2], xy[3]) {
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

  void extend(const vector3_type &v) {
    min = vdb::math::minComponent(min, vector_type(v.x(), v.y()));
    max = vdb::math::maxComponent(max, vector_type(v.x(), v.y()));
  }

  void extend(const vector_type &v) {
    min = vdb::math::minComponent(min, v);
    max = vdb::math::maxComponent(max, v);
  }

  void extend(const bbox2 &box) {
    min = vdb::math::minComponent(min, box.min);
    max = vdb::math::maxComponent(max, box.max);
  }

  void expand(const vector_type &xyMin, const scalar_type &length) {
    const scalar_type size =
        std::is_integral<scalar_type>::value ? length - 1 : length;
#pragma unroll
    for (int i = 0; i < 2; ++i) {
      min[i] = std::min(min[i], xyMin[i]);
      max[i] = std::max(max[i], xyMin[i] + size);
    }
  }

  void scale(scalar_type x) noexcept {
    vector_type const v = x * halfsize();
    min -= v;
    max += v;
  }

  scalar_type width() const { return max.x() - min.x(); }

  scalar_type height() const { return max.y() - min.y(); }

  scalar_type area() const {
    vector_type d = max - min;
    return d.x() * d.y();
  }

  void translate(const vector_type &dx) {
    min += dx;
    max += dx;
  }

  void sort() {
    vector_type tMin(min), tMax(max);
#pragma unroll
    for (int i = 0; i < 2; ++i) {
      min[i] = std::min(tMin[i], tMax[i]);
      max[i] = std::max(tMin[i], tMax[i]);
    }
  }

  bool is_sorted() const {
    if (std::is_integral<scalar_type>::value) {
      return (min.x() <= max.x() && min.y() <= max.y());
    } else {
      scalar_type t = tolerance<scalar_type>();
      return (min.x() < (max.x() + t) && min.y() < (max.y() + t));
    }
  }

  vector_type extents() const {
    if (std::is_integral<scalar_type>::value) {
      return (max - min) + vector_type(1, 1);
    } else {
      return (max - min);
    }
  }

  vector_type diagonal() const { return max - min; }

  vector_type center() const { return (min + max) * 0.5f; }

  vector_type center2() const { return (min + max); }

  vector_type halfsize() const { return 0.5f * (max - min); }

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

  bool eq(const bbox2 &rhs) const { return (*this == rhs); }

  bool operator==(const bbox2 &rhs) const {
    if (std::is_integral<scalar_type>::value) {
      return min == rhs.min && max == rhs.max;
    } else {
      return vdb::math::isApproxEqual(min, rhs.min) &&
             vdb::math::isApproxEqual(max, rhs.max);
    }
  }

  bool operator!=(const bbox2 &rhs) const { return !(*this == rhs); }

  bool empty() const {
    if (std::is_integral<scalar_type>::value) {
      return (min.x() > max.x() || min.y() > max.y());
    }
    return min.x() >= max.x() || min.y() >= max.y();
  }

  vector_type intersect(const ray2<scalar_type> &r) const {
    vector_type v1 = min, v2 = max;
    const vector_type bbox_eps{0.0000001f};

    vector_type div = r.direction;

    if (std::abs(div.x()) < bbox_eps.x() && std::abs(div.y()) < bbox_eps.y()) {
      div = bbox_eps;
    }

    vector_type t1 = (v1 - r.origin) / div;
    vector_type t2 = (v2 - r.origin) / div;

    vector_type tmin = vdb::math::minComponent(t1, t2);
    vector_type tmax = vdb::math::maxComponent(t1, t2);

    return {std::max(tmin.x(), tmin.y()), std::min(tmax.x(), tmax.y())};
  }

  bool contains(const vector_type &p) const {
    if (std::is_integral<scalar_type>::value) {
      if (((p.x() >= min.x() && p.x() <= max.x()) ||
           (p.x() <= min.x() && p.x() >= max.x())) &&
          ((p.y() >= min.y() && p.y() <= max.y()) ||
           (p.y() <= min.y() && p.y() >= max.y()))) {
        return true;
      }
      return false;
    } else {
      scalar_type t = tolerance<scalar_type>();
      if (((p.x() >= (min.x() - t) && p.x() <= (max.x() - t)) ||
           (p.x() <= (min.x() - t) && p.x() >= (max.x() - t))) &&
          ((p.y() >= (min.y() - t) && p.y() <= (max.y() - t)) ||
           (p.y() <= (min.y() - t) && p.y() >= (max.y() - t)))) {
        return true;
      }
      return false;
    }
  }

  bool contains(const bbox2 &b) const {
    if (std::is_integral<scalar_type>::value) {
      return b.min.x() >= min.x() && b.max.x() <= max.x() &&
             b.min.y() >= min.y() && b.max.y() <= max.y();
    } else {
      scalar_type t = tolerance<scalar_type>();
      return (b.min.x() - t) > min.x() && (b.max.x() + t) < max.x() &&
             (b.min.y() - t) > min.y() && (b.max.y() + t) < max.y();
    }
  }

  bool overlap(const bbox2 &b) const {
    if (std::is_integral<scalar_type>::value) {
      return max.x() >= b.min.x() && min.x() <= b.max.x() &&
             max.y() >= b.min.y() && min.y() <= b.max.y();
    } else {
      scalar_type t = tolerance<scalar_type>();
      return max.x() > (b.min.x() - t) && min.x() < (b.max.x() + t) &&
             max.y() > (b.min.y() - t) && min.y() < (b.max.y() + t);
    }
  }
};

template <typename T> bbox2<T> operator*(const float &a, const bbox2<T> &b) {
  return bbox2<T>(a * b.min, a * b.max);
}

template <typename T>
bbox2<T> operator*(const vdb::math::Vec2<T> &a,
                   const bbox2<T> &b) {
  return bbox2<T>(a * b.min, a * b.max);
}

template <typename T> bbox2<T> operator+(const bbox2<T> &a, const bbox2<T> &b) {
  return bbox2<T>(a.min + b.min, a.max + b.max);
}

template <typename T> bbox2<T> operator-(const bbox2<T> &a, const bbox2<T> &b) {
  return bbox2<T>(a.min - b.min, a.max - b.max);
}

template <typename T>
bbox2<T> operator+(const bbox2<T> &a,
                   const vdb::math::Vec2<T> &b) {
  return bbox2<T>(a.min + b, a.max + b);
}

template <typename T>
bbox2<T> operator-(const bbox2<T> &a,
                   const vdb::math::Vec2<T> &b) {
  return bbox2<T>(a.min - b, a.max - b);
}

template <typename T>
bbox2<T> enlarge(const bbox2<T> &a,
                 const vdb::math::Vec2<T> &b) {
  return bbox2<T>(a.min - b, a.max + b);
}

template <typename T>
const bbox2<T> merge(const bbox2<T> &a,
                     const vdb::math::Vec2<T> &b) {
  return bbox2<T>(vdb::math::minComponent(a.min, b),
                  vdb::math::maxComponent(a.max, b));
}

template <typename T>
const bbox2<T> merge(const vdb::math::Vec2<T> &a,
                     const bbox2<T> &b) {
  return bbox2<T>(vdb::math::minComponent(a, b.min),
                  vdb::math::maxComponent(a, b.max));
}

template <typename T>
const bbox2<T> merge(const bbox2<T> &a, const bbox2<T> &b) {
  return bbox2<T>(vdb::math::minComponent(a.min, b.min),
                  vdb::math::maxComponent(a.max, b.max));
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
  return bbox2<T>(vdb::math::maxComponent(a.min, b.min),
                  vdb::math::minComponent(a.max, b.max));
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
bool disjoint(const bbox2<T> &a,
              const vdb::math::Vec2<T> &b) {
  return disjoint(a, bbox2<T>(b));
}

template <typename T>
bool disjoint(const vdb::math::Vec2<T> &a,
              const bbox2<T> &b) {
  return disjoint(bbox2<T>(a), b);
}

template <typename T> bool conjoint(const bbox2<T> &a, const bbox2<T> &b) {
  return !intersect(a, b).empty();
}

template <typename T>
bool conjoint(const bbox2<T> &a,
              const vdb::math::Vec2<T> &b) {
  return conjoint(a, bbox2<T>(b));
}

template <typename T>
bool conjoint(const vdb::math::Vec2<T> &a,
              const bbox2<T> &b) {
  return conjoint(bbox2<T>(a), b);
}

template <typename T>
void subtract(const bbox2<T> &a, const bbox2<T> &b, bbox2<T> &c, bbox2<T> &d) {
  c.min = a.min;
  c.max = vdb::math::minComponent(a.max, b.min);
  d.min = vdb::math::maxComponent(a.min, b.max);
  d.max = a.max;
}

template <typename T> bool subset(const bbox2<T> &a, const bbox2<T> &b) {
  for (size_t i = 0; i < T::N; i++)
    if (a.min(i) < b.min(i))
      return false;
  for (size_t i = 0; i < T::N; i++)
    if (a.max(i) > b.max(i))
      return false;
  return true;
}

} // namespace flywave
