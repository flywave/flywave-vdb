#pragma once

#include <openvdb/Types.h>

namespace flywave {

template <typename Color> class texture2d;

namespace impl {

template <typename ColorType> class texture_row_iterator;

template <typename ColorType> class const_texture_row_iterator;

template <typename ColorType> class texture_row {
public:
  ~texture_row() = default;
  texture_row(const texture_row<ColorType> &) = default;
  texture_row &operator=(const texture_row<ColorType> &) = default;
  texture_row(texture_row<ColorType> &&) noexcept = default;
  texture_row &operator=(texture_row<ColorType> &&) noexcept = default;

  ColorType *begin() noexcept { return img_->color_begin(row_index_); }

  const ColorType *begin() const noexcept {
    return img_->color_begin(row_index_);
  }

  const ColorType *cbegin() const noexcept {
    return img_->color_begin(row_index_);
  }

  ColorType *end() noexcept { return img_->color_end(row_index_); }

  const ColorType *end() const noexcept { return img_->color_end(row_index_); }

  const ColorType *cend() const noexcept { return img_->color_end(row_index_); }

  size_t index() const noexcept { return row_index_; }

  bool operator==(const texture_row &it) const noexcept {
    return row_index_ == it.row_index_ && img_ == it.img_;
  }

  bool operator!=(const texture_row &it) const noexcept {
    return row_index_ != it.row_index_ || img_ != it.img_;
  }

private:
  texture2d<ColorType> *img_;
  size_t row_index_;

  texture_row(texture2d<ColorType> *img, size_t row_index)
      : img_(img), row_index_(row_index) {}

  friend class texture2d<ColorType>;
  friend class texture_row_iterator<ColorType>;
};

template <typename ColorType> class texture_row_iterator {
public:
  ~texture_row_iterator() = default;
  texture_row_iterator(const texture_row_iterator<ColorType> &) = default;
  texture_row_iterator &
  operator=(const texture_row_iterator<ColorType> &) = default;
  texture_row_iterator(texture_row_iterator<ColorType> &&) noexcept = default;
  texture_row_iterator &
  operator=(texture_row_iterator<ColorType> &&) noexcept = default;

  texture_row_iterator<ColorType> &operator--() noexcept {
    --row_.row_index_;
    return *this;
  }

  texture_row_iterator<ColorType> &operator++() noexcept {
    ++row_.row_index_;
    return *this;
  }

  texture_row_iterator<ColorType> operator--(int) noexcept {
    texture_row_iterator it(*this);
    operator--();
    return it;
  }

  texture_row_iterator<ColorType> operator++(int) noexcept {
    texture_row_iterator it(*this);
    operator++();
    return it;
  }

  bool operator==(const texture_row_iterator<ColorType> &it) const noexcept {
    return row_ == it.row_;
  }

  bool operator!=(const texture_row_iterator<ColorType> &it) const noexcept {
    return row_ != it.row_;
  }

  texture_row<ColorType> &operator*() noexcept { return row_; }

private:
  texture_row<ColorType> row_;

  explicit texture_row_iterator(texture_row<ColorType> row) : row_(row) {}

  friend class texture2d<ColorType>;
};

template <typename ColorType> class const_texture_row {
public:
  ~const_texture_row() = default;
  const_texture_row(const const_texture_row<ColorType> &) = default;
  const_texture_row &operator=(const const_texture_row<ColorType> &) = default;
  const_texture_row(const_texture_row<ColorType> &&) noexcept = default;
  const_texture_row &
  operator=(const_texture_row<ColorType> &&) noexcept = default;

  const ColorType *begin() const noexcept {
    return img_->color_begin(row_index_);
  }

  const ColorType *cbegin() const noexcept {
    return img_->color_begin(row_index_);
  }

  const ColorType *end() const noexcept { return img_->color_end(row_index_); }

  const ColorType *cend() const noexcept { return img_->color_end(row_index_); }

  size_t index() const noexcept { return row_index_; }

  bool operator==(const const_texture_row &it) const noexcept {
    return row_index_ == it.row_index_ && img_ == it.img_;
  }

  bool operator!=(const const_texture_row &it) const noexcept {
    return row_index_ != it.row_index_ || img_ != it.img_;
  }

private:
  const texture2d<ColorType> *img_;
  size_t row_index_;

  const_texture_row(const texture2d<ColorType> *img, size_t row_index)
      : img_(img), row_index_(row_index) {}

  friend class texture2d<ColorType>;
  friend class const_texture_row_iterator<ColorType>;
};

template <typename ColorType> class const_texture_row_iterator {
public:
  ~const_texture_row_iterator() = default;
  const_texture_row_iterator(const const_texture_row_iterator<ColorType> &) =
      default;
  const_texture_row_iterator &
  operator=(const const_texture_row_iterator<ColorType> &) = default;
  const_texture_row_iterator(
      const_texture_row_iterator<ColorType> &&) noexcept = default;
  const_texture_row_iterator &
  operator=(const_texture_row_iterator<ColorType> &&) noexcept = default;

  const_texture_row_iterator<ColorType> &operator--() noexcept {
    --row_.row_index_;
    return *this;
  }

  const_texture_row_iterator<ColorType> &operator++() noexcept {
    ++row_.row_index_;
    return *this;
  }

  const_texture_row_iterator<ColorType> operator--(int) noexcept {
    const_texture_row_iterator it(*this);
    operator--();
    return it;
  }

  const_texture_row_iterator<ColorType> operator++(int) noexcept {
    const_texture_row_iterator it(*this);
    operator++();
    return it;
  }

  bool
  operator==(const const_texture_row_iterator<ColorType> &it) const noexcept {
    return row_ == it.row_;
  }

  bool
  operator!=(const const_texture_row_iterator<ColorType> &it) const noexcept {
    return row_ != it.row_;
  }

  const const_texture_row<ColorType> &operator*() const noexcept {
    return row_;
  }

private:
  const_texture_row<ColorType> row_;

  explicit const_texture_row_iterator(const_texture_row<ColorType> row)
      : row_(row) {}

  friend class texture2d<ColorType>;
};

} // namespace impl

template <typename T, typename S = T> struct color_value_type {
  typedef T pixel_type;
  typedef T type;
  typedef S vector_type;
};

template <typename T, typename S>
struct color_value_type<openvdb::math::Vec2<T>, S> {
  typedef T pixel_type;
  typedef openvdb::math::Vec2<T> type;
  typedef openvdb::math::Vec2<S> vector_type;
};

template <typename T, typename S>
struct color_value_type<openvdb::math::Vec3<T>, S> {
  typedef T pixel_type;
  typedef openvdb::math::Vec3<T> type;
  typedef openvdb::math::Vec3<S> vector_type;
};

template <typename T, typename S>
struct color_value_type<openvdb::math::Vec4<T>, S> {
  typedef T pixel_type;
  typedef openvdb::math::Vec4<T> type;
  typedef openvdb::math::Vec4<S> vector_type;
};

template <typename C> C pixel_get(const C &c, int);
template <typename C> C pixel_get(const openvdb::math::Vec2<C> &c, int i);
template <typename C> C pixel_get(const openvdb::math::Vec3<C> &c, int i);
template <typename C> C pixel_get(const openvdb::math::Vec4<C> &c, int i);

template <typename C> C &pixel_get(C &c, int);
template <typename C> C &pixel_get(openvdb::math::Vec2<C> &c, int i);
template <typename C> C &pixel_get(openvdb::math::Vec3<C> &c, int i);
template <typename C> C &pixel_get(openvdb::math::Vec4<C> &c, int i);

template <> uint8_t pixel_get<uint8_t>(const uint8_t &c, int) { return c; }

template <> uint16_t pixel_get<uint16_t>(const uint16_t &c, int) { return c; }

template <> float pixel_get<float>(const float &c, int) { return c; }

template <>
uint8_t pixel_get<uint8_t>(const openvdb::math::Vec2<uint8_t> &c, int i) {
  return c(i);
}
template <>
uint16_t pixel_get<uint16_t>(const openvdb::math::Vec2<uint16_t> &c, int i) {
  return c(i);
}
template <> float pixel_get<float>(const openvdb::math::Vec2<float> &c, int i) {
  return c(i);
}

template <>
uint8_t pixel_get<uint8_t>(const openvdb::math::Vec3<uint8_t> &c, int i) {
  return c(i);
}
template <>
uint16_t pixel_get<uint16_t>(const openvdb::math::Vec3<uint16_t> &c, int i) {
  return c(i);
}
template <> float pixel_get<float>(const openvdb::math::Vec3<float> &c, int i) {
  return c(i);
}

template <>
uint8_t pixel_get<uint8_t>(const openvdb::math::Vec4<uint8_t> &c, int i) {
  return c(i);
}
template <>
uint16_t pixel_get<uint16_t>(const openvdb::math::Vec4<uint16_t> &c, int i) {
  return c(i);
}
template <> float pixel_get<float>(const openvdb::math::Vec4<float> &c, int i) {
  return c(i);
}

template <> uint8_t &pixel_get<uint8_t>(uint8_t &c, int) { return c; }

template <> uint16_t &pixel_get<uint16_t>(uint16_t &c, int) { return c; }

template <> float &pixel_get<float>(float &c, int) { return c; }

template <>
uint8_t &pixel_get<uint8_t>(openvdb::math::Vec2<uint8_t> &c, int i) {
  return c(i);
}
template <>
uint16_t &pixel_get<uint16_t>(openvdb::math::Vec2<uint16_t> &c, int i) {
  return c(i);
}
template <> float &pixel_get<float>(openvdb::math::Vec2<float> &c, int i) {
  return c(i);
}

template <>
uint8_t &pixel_get<uint8_t>(openvdb::math::Vec3<uint8_t> &c, int i) {
  return c(i);
}
template <>
uint16_t &pixel_get<uint16_t>(openvdb::math::Vec3<uint16_t> &c, int i) {
  return c(i);
}
template <> float &pixel_get<float>(openvdb::math::Vec3<float> &c, int i) {
  return c(i);
}

template <>
uint8_t &pixel_get<uint8_t>(openvdb::math::Vec4<uint8_t> &c, int i) {
  return c(i);
}
template <>
uint16_t &pixel_get<uint16_t>(openvdb::math::Vec4<uint16_t> &c, int i) {
  return c(i);
}
template <> float &pixel_get<float>(openvdb::math::Vec4<float> &c, int i) {
  return c(i);
}

template <typename V> struct color_channel {
  constexpr operator size_t() const { return size_t(1); }
};

template <typename T> struct color_channel<openvdb::math::Vec2<T>> {
  constexpr operator size_t() const { return size_t(2); }
};

template <typename T> struct color_channel<openvdb::math::Vec3<T>> {
  constexpr operator size_t() const { return size_t(3); }
};

template <typename T> struct color_channel<openvdb::math::Vec4<T>> {
  constexpr operator size_t() const { return size_t(4); }
};

template <class Scalar, class ScalarInterpType>
void lerp(openvdb::math::Vec2<Scalar> &c, openvdb::math::Vec2<Scalar> c0,
          openvdb::math::Vec2<Scalar> c1, ScalarInterpType x) {
  assert(x >= 0);
  assert(x <= 1);

  c(0) = (Scalar)(c1(0) * x + c0(0) * (1.0f - x));
  c(1) = (Scalar)(c1(1) * x + c0(1) * (1.0f - x));
}

template <class Scalar, class ScalarInterpType>
void lerp_range(openvdb::math::Vec2<Scalar> &c, openvdb::math::Vec2<Scalar> c0,
                openvdb::math::Vec2<Scalar> c1, openvdb::math::Vec2<Scalar> c2,
                openvdb::math::Vec2<ScalarInterpType> ip) {

  assert(std::abs(ip(0) + ip(1) + ip(2) - 1) < 0.00001);

  c(0) = (Scalar)(c0(0) * ip(0) + c1(0) * ip(1) + c2(0) * ip(2));
  c(1) = (Scalar)(c0(1) * ip(0) + c1(1) * ip(1) + c2(1) * ip(2));
}

template <class Scalar, class ScalarInterpType>
void lerp_cube(openvdb::math::Vec2<Scalar> &c, openvdb::math::Vec2<Scalar> c0,
               openvdb::math::Vec2<Scalar> c1, openvdb::math::Vec2<Scalar> c2,
               openvdb::math::Vec2<Scalar> c3,
               openvdb::math::Vec4<ScalarInterpType> ip) {

  assert(std::abs(ip(0) + ip(1) + ip(2) + ip(3) - 1) < 0.00001);

  c(0) =
      (Scalar)(c0(0) * ip(0) + c1(0) * ip(1) + c2(0) * ip(2) + c3(0) * ip(3));
  c(1) =
      (Scalar)(c0(1) * ip(0) + c1(1) * ip(1) + c2(1) * ip(2) + c3(1) * ip(3));
}

template <class Scalar, class ScalarInterpType>
void lerp(openvdb::math::Vec3<Scalar> &c, openvdb::math::Vec3<Scalar> c0,
          openvdb::math::Vec3<Scalar> c1, ScalarInterpType x) {
  assert(x >= 0);
  assert(x <= 1);

  c(0) = (Scalar)(c1(0) * x + c0(0) * (1.0f - x));
  c(1) = (Scalar)(c1(1) * x + c0(1) * (1.0f - x));
  c(2) = (Scalar)(c1(2) * x + c0(2) * (1.0f - x));
}

template <class Scalar, class ScalarInterpType>
void lerp_range(openvdb::math::Vec3<Scalar> &c, openvdb::math::Vec3<Scalar> c0,
                openvdb::math::Vec3<Scalar> c1, openvdb::math::Vec3<Scalar> c2,
                openvdb::math::Vec3<ScalarInterpType> ip) {

  assert(std::abs(ip(0) + ip(1) + ip(2) - 1) < 0.00001);

  c(0) = (Scalar)(c0(0) * ip(0) + c1(0) * ip(1) + c2(0) * ip(2));
  c(1) = (Scalar)(c0(1) * ip(0) + c1(1) * ip(1) + c2(1) * ip(2));
  c(2) = (Scalar)(c0(2) * ip(0) + c1(2) * ip(1) + c2(2) * ip(2));
}

template <class Scalar, class ScalarInterpType>
void lerp_cube(openvdb::math::Vec3<Scalar> &c, openvdb::math::Vec3<Scalar> c0,
               openvdb::math::Vec3<Scalar> c1, openvdb::math::Vec3<Scalar> c2,
               openvdb::math::Vec3<Scalar> c3,
               openvdb::math::Vec4<ScalarInterpType> ip) {

  assert(std::abs(ip(0) + ip(1) + ip(2) + ip(3) - 1) < 0.00001);

  c(0) =
      (Scalar)(c0(0) * ip(0) + c1(0) * ip(1) + c2(0) * ip(2) + c3(0) * ip(3));
  c(1) =
      (Scalar)(c0(1) * ip(0) + c1(1) * ip(1) + c2(1) * ip(2) + c3(1) * ip(3));
  c(2) =
      (Scalar)(c0(2) * ip(0) + c1(2) * ip(1) + c2(2) * ip(2) + c3(2) * ip(3));
}

template <class Scalar, class ScalarInterpType>
openvdb::math::Vec4<Scalar>
lerp(openvdb::math::Vec4<Scalar> &c, openvdb::math::Vec4<Scalar> c0,
     openvdb::math::Vec4<Scalar> c1, ScalarInterpType x) {
  c(0) = (Scalar)(c1(0) * x + c0(0) * (1.0f - x));
  c(1) = (Scalar)(c1(1) * x + c0(1) * (1.0f - x));
  c(2) = (Scalar)(c1(2) * x + c0(2) * (1.0f - x));
  c(3) = (Scalar)(c1(3) * x + c0(3) * (1.0f - x));
}

template <class Scalar, class ScalarInterpType>
void lerp_range(openvdb::math::Vec4<Scalar> &c, openvdb::math::Vec4<Scalar> c0,
                openvdb::math::Vec4<Scalar> c1, openvdb::math::Vec4<Scalar> c2,
                openvdb::math::Vec3<ScalarInterpType> ip) {
  c(0) = (Scalar)(c0(0) * ip(0) + c1(0) * ip(1) + c2(0) * ip(2));
  c(1) = (Scalar)(c0(1) * ip(0) + c1(1) * ip(1) + c2(1) * ip(2));
  c(2) = (Scalar)(c0(2) * ip(0) + c1(2) * ip(1) + c2(2) * ip(2));
  c(3) = (Scalar)(c0(3) * ip(0) + c1(3) * ip(1) + c2(3) * ip(2));
}

template <class Scalar, class ScalarInterpType>
void lerp_cube(openvdb::math::Vec4<Scalar> &c, openvdb::math::Vec4<Scalar> c0,
               openvdb::math::Vec4<Scalar> c1, openvdb::math::Vec4<Scalar> c2,
               openvdb::math::Vec4<Scalar> c3,
               openvdb::math::Vec4<ScalarInterpType> ip) {
  c(0) =
      (Scalar)(c0(0) * ip(0) + c1(0) * ip(1) + c2(0) * ip(2) + c3(0) * ip(3));
  c(1) =
      (Scalar)(c0(1) * ip(0) + c1(1) * ip(1) + c2(1) * ip(2) + c3(1) * ip(3));
  c(2) =
      (Scalar)(c0(2) * ip(0) + c1(2) * ip(1) + c2(2) * ip(2) + c3(2) * ip(3));
  c(3) =
      (Scalar)(c0(3) * ip(0) + c1(3) * ip(1) + c2(3) * ip(2) + c3(3) * ip(3));
}

template <typename T, typename ScalarInterpType>
inline constexpr T interpolate(T const &v1, ScalarInterpType w1) {
  return v1 * w1;
}

template <typename ScalarInterpType>
inline constexpr unsigned char interpolate(unsigned char const &v1,
                                           ScalarInterpType w1) {
  return (unsigned char)((ScalarInterpType)v1 * w1 + 0.5f);
}

template <typename T, typename ScalarInterpType>
inline constexpr T interpolate(T const &v1, T const &v2, ScalarInterpType w1) {
  return v1 * w1 + v2 * (1.0f - w1);
}

template <typename ScalarInterpType>
inline constexpr unsigned char interpolate(unsigned char const &v1,
                                           unsigned char const &v2,
                                           ScalarInterpType w1) {
  return (unsigned char)((ScalarInterpType)v1 * w1 +
                         (ScalarInterpType)v2 * (1.0f - w1) + 0.5f);
}

template <typename T, typename ScalarInterpType>
inline constexpr T interpolate(T const &v1, T const &v2, ScalarInterpType w1,
                               ScalarInterpType w2) {
  return v1 * w1 + v2 * w2;
}

template <typename ScalarInterpType>
inline constexpr unsigned char
interpolate(unsigned char const &v1, unsigned char const &v2,
            ScalarInterpType w1, ScalarInterpType w2) {
  return (unsigned char)((ScalarInterpType)v1 * w1 + (ScalarInterpType)v2 * w2 +
                         0.5f);
}

template <typename T, typename ScalarInterpType>
inline constexpr T interpolate(T const &v1, T const &v2,
                               const openvdb::math::Vec2<ScalarInterpType> &t) {
  return v1 * t.x() + v2 * t.y();
}

template <typename ScalarInterpType>
inline constexpr unsigned char
interpolate(unsigned char const &v1, unsigned char const &v2,
            const openvdb::math::Vec2<ScalarInterpType> &t) {
  return (unsigned char)((ScalarInterpType)v1 * t.x() +
                         (ScalarInterpType)v2 * t.y() + 0.5f);
}

template <typename T, typename ScalarInterpType>
inline constexpr T interpolate(T const &v1, T const &v2, T const &v3,
                               ScalarInterpType w1, ScalarInterpType w2,
                               ScalarInterpType w3) {
  return v1 * w1 + v2 * w2 + v3 * w3;
}

template <typename ScalarInterpType>
inline constexpr unsigned char
interpolate(unsigned char const &v1, unsigned char const &v2,
            unsigned char const &v3, ScalarInterpType w1, ScalarInterpType w2,
            ScalarInterpType w3) {
  return (unsigned char)((ScalarInterpType)v1 * w1 + (ScalarInterpType)v2 * w2 +
                         (ScalarInterpType)v3 * w3 + 0.5f);
}

template <typename T, typename ScalarInterpType>
inline constexpr T interpolate(T const &v1, T const &v2, T const &v3,
                               const openvdb::math::Vec3<ScalarInterpType> &t) {
  return v1 * t.x() + v2 * t.y() + v3 * t.z();
}

template <typename ScalarInterpType>
inline constexpr unsigned char
interpolate(unsigned char const &v1, unsigned char const &v2,
            unsigned char const &v3,
            const openvdb::math::Vec3<ScalarInterpType> &t) {
  return (unsigned char)((ScalarInterpType)v1 * t.x() +
                         (ScalarInterpType)v2 * t.y() +
                         (ScalarInterpType)v3 * t.z() + 0.5f);
}

template <typename T, typename ScalarInterpType>
inline constexpr T interpolate(T const &v1, T const &v2, T const &v3,
                               T const &v4, ScalarInterpType w1,
                               ScalarInterpType w2, ScalarInterpType w3,
                               ScalarInterpType w4) {
  return v1 * w1 + v2 * w2 + v3 * w3 + v4 * w4;
}

template <typename ScalarInterpType>
inline constexpr unsigned char
interpolate(unsigned char const &v1, unsigned char const &v2,
            unsigned char const &v3, unsigned char const &v4,
            ScalarInterpType w1, ScalarInterpType w2, ScalarInterpType w3,
            ScalarInterpType w4) {
  return (unsigned char)((ScalarInterpType)v1 * w1 + (ScalarInterpType)v2 * w2 +
                         (ScalarInterpType)v3 * w3 + (ScalarInterpType)v4 * w4 +
                         0.5f);
}

template <typename T, typename ScalarInterpType>
inline T interpolate(T const &v1, T const &v2, T const &v3, T const &v4,
                     const openvdb::math::Vec4<ScalarInterpType> &t) {
  return v1 * t.x() + v2 * t.y() + v3 * t.z() + v4 * t.w();
}

template <typename ScalarInterpType>
inline unsigned char
interpolate(unsigned char const &v1, unsigned char const &v2,
            unsigned char const &v3, unsigned char const &v4,
            const openvdb::math::Vec4<ScalarInterpType> &t) {
  return (unsigned char)((ScalarInterpType)v1 * t.x() +
                         (ScalarInterpType)v2 * t.y() +
                         (ScalarInterpType)v3 * t.z() +
                         (ScalarInterpType)v4 * t.w() + 0.5f);
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec2<fp_type>
interpolate(const openvdb::math::Vec2<fp_type> &a,
            const openvdb::math::Vec2<fp_type> &b, const ScalarInterpType &t) {
  openvdb::math::Vec2<fp_type> e;
  lerp(e, a, b, t);
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec3<fp_type>
interpolate(const openvdb::math::Vec3<fp_type> &a,
            const openvdb::math::Vec3<fp_type> &b, const ScalarInterpType &t) {
  openvdb::math::Vec3<fp_type> e;
  lerp(e, a, b, t);
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec4<fp_type>
interpolate(const openvdb::math::Vec4<fp_type> &a,
            const openvdb::math::Vec4<fp_type> &b, const ScalarInterpType &t) {
  openvdb::math::Vec4<fp_type> e;
  lerp(e, a, b, t);
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec2<fp_type>
interpolate(const openvdb::math::Vec2<fp_type> &a,
            const openvdb::math::Vec2<fp_type> &b,
            const openvdb::math::Vec2<fp_type> &c,
            const openvdb::math::Vec3<ScalarInterpType> &t) {
  openvdb::math::Vec2<fp_type> e;
  lerp_range(e, a, b, c, t);
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec3<fp_type>
interpolate(const openvdb::math::Vec3<fp_type> &a,
            const openvdb::math::Vec3<fp_type> &b,
            const openvdb::math::Vec3<fp_type> &c,
            const openvdb::math::Vec3<ScalarInterpType> &t) {
  openvdb::math::Vec3<fp_type> e;
  lerp_range(e, a, b, c, t);
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec4<fp_type>
interpolate(const openvdb::math::Vec4<fp_type> &a,
            const openvdb::math::Vec4<fp_type> &b,
            const openvdb::math::Vec4<fp_type> &c,
            const openvdb::math::Vec3<ScalarInterpType> &t) {
  openvdb::math::Vec4<fp_type> e;
  lerp_range(e, a, b, c, t);
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec2<fp_type>
interpolate(const openvdb::math::Vec2<fp_type> &a,
            const openvdb::math::Vec2<fp_type> &b,
            const openvdb::math::Vec2<fp_type> &c, ScalarInterpType w1,
            ScalarInterpType w2, ScalarInterpType w3) {
  openvdb::math::Vec2<fp_type> e;
  lerp_range(e, a, b, c, openvdb::math::Vec3<ScalarInterpType>{w1, w2, w3});
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec3<fp_type>
interpolate(const openvdb::math::Vec3<fp_type> &a,
            const openvdb::math::Vec3<fp_type> &b,
            const openvdb::math::Vec3<fp_type> &c, ScalarInterpType w1,
            ScalarInterpType w2, ScalarInterpType w3) {
  openvdb::math::Vec3<fp_type> e;
  lerp_range(e, a, b, c, openvdb::math::Vec3<ScalarInterpType>{w1, w2, w3});
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec4<fp_type>
interpolate(const openvdb::math::Vec4<fp_type> &a,
            const openvdb::math::Vec4<fp_type> &b,
            const openvdb::math::Vec4<fp_type> &c, ScalarInterpType w1,
            ScalarInterpType w2, ScalarInterpType w3) {
  openvdb::math::Vec4<fp_type> e;
  lerp_range(e, a, b, c, openvdb::math::Vec3<ScalarInterpType>{w1, w2, w3});
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec2<fp_type>
interpolate(const openvdb::math::Vec2<fp_type> &a,
            const openvdb::math::Vec2<fp_type> &b,
            const openvdb::math::Vec2<fp_type> &c,
            const openvdb::math::Vec2<fp_type> &d,
            const openvdb::math::Vec4<ScalarInterpType> &t) {
  openvdb::math::Vec2<fp_type> e;
  lerp_cube(e, a, b, c, d, t);
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec3<fp_type>
interpolate(const openvdb::math::Vec3<fp_type> &a,
            const openvdb::math::Vec3<fp_type> &b,
            const openvdb::math::Vec3<fp_type> &c,
            const openvdb::math::Vec3<fp_type> &d,
            const openvdb::math::Vec4<ScalarInterpType> &t) {
  openvdb::math::Vec3<fp_type> e;
  lerp_cube(e, a, b, c, d, t);
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec4<fp_type>
interpolate(const openvdb::math::Vec4<fp_type> &a,
            const openvdb::math::Vec4<fp_type> &b,
            const openvdb::math::Vec4<fp_type> &c,
            const openvdb::math::Vec4<fp_type> &d,
            const openvdb::math::Vec4<ScalarInterpType> &t) {
  openvdb::math::Vec4<fp_type> e;
  lerp_cube(e, a, b, c, d, t);
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec2<fp_type>
interpolate(const openvdb::math::Vec2<fp_type> &a,
            const openvdb::math::Vec2<fp_type> &b,
            const openvdb::math::Vec2<fp_type> &c,
            const openvdb::math::Vec2<fp_type> &d, ScalarInterpType w1,
            ScalarInterpType w2, ScalarInterpType w3, ScalarInterpType w4) {
  openvdb::math::Vec2<fp_type> e;
  lerp_cube(e, a, b, c, d,
            openvdb::math::Vec4<ScalarInterpType>{w1, w2, w3, w4});
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec3<fp_type>
interpolate(const openvdb::math::Vec3<fp_type> &a,
            const openvdb::math::Vec3<fp_type> &b,
            const openvdb::math::Vec3<fp_type> &c,
            const openvdb::math::Vec3<fp_type> &d, ScalarInterpType w1,
            ScalarInterpType w2, ScalarInterpType w3, ScalarInterpType w4) {
  openvdb::math::Vec3<fp_type> e;
  lerp_cube(e, a, b, c, d,
            openvdb::math::Vec4<ScalarInterpType>{w1, w2, w3, w4});
  return e;
}

template <typename fp_type, typename ScalarInterpType>
constexpr openvdb::math::Vec4<fp_type>
interpolate(const openvdb::math::Vec4<fp_type> &a,
            const openvdb::math::Vec4<fp_type> &b,
            const openvdb::math::Vec4<fp_type> &c,
            const openvdb::math::Vec4<fp_type> &d, ScalarInterpType w1,
            ScalarInterpType w2, ScalarInterpType w3, ScalarInterpType w4) {
  openvdb::math::Vec4<fp_type> e;
  lerp_cube(e, a, b, c, d,
            openvdb::math::Vec4<ScalarInterpType>{w1, w2, w3, w4});
  return e;
}

template <typename Color>
class texture2d : public std::enable_shared_from_this<base_texture> {
public:
  typedef std::shared_ptr<texture2d<Color>> Ptr;
  typedef const std::shared_ptr<texture2d<Color>> ConstPtr;
  typedef typename color_value_type<Color>::pixel_type PixelType;
  typedef Color color_type;

  typedef impl::texture_row_iterator<Color> Iterator;
  typedef impl::const_texture_row_iterator<Color> ConstIterator;

  typedef std::vector<uint8_t *> RowPointersType;
  typedef std::vector<const uint8_t *> ConstRowPointersType;

  std::pair<uint32_t, uint32_t> size;
  static constexpr size_t channels_ = color_channel<Color>();
  static constexpr size_t color_size = sizeof(Color);
  std::unique_ptr<uint8_t[]> data;

public:
  texture2d() = default;
  texture2d(const texture2d &img) = delete;
  texture2d(std::pair<uint32_t, uint32_t> size_) : size(size_) {
    data = std::make_unique<uint8_t[]>(bytes());
  }

  texture2d(std::pair<uint32_t, uint32_t> size_, const uint8_t *srcData,
            size_t srcLength)
      : size(size_) {
    if (srcLength != bytes()) {
      throw std::invalid_argument("mismatched image size");
    }
    data = std::make_unique<uint8_t[]>(bytes());
    std::copy(srcData, srcData + srcLength, data.get());
  }

  texture2d(std::pair<uint32_t, uint32_t> size_,
            std::unique_ptr<uint8_t[]> data_)
      : base_texture(std::move(data_)), size(size_) {}

  texture2d(texture2d &&o) : base_texture(std::move(o.data)), size(o.size) {
    o.size.first = o.size.second = 0;
  }

  texture2d &operator=(texture2d &&o) {
    size = o.size;
    data = std::move(o.data);
    o.size.first = o.size.second = 0;
    return *this;
  }

  void allocate(std::pair<uint32_t, uint32_t> size_) {
    data.reset();
    size = size_;
    data = std::make_unique<uint8_t[]>(bytes());
  }

  static std::shared_ptr<texture2d<Color>> create() {
    return std::make_shared<texture2d<Color>>();
  }

  static std::shared_ptr<texture2d<Color>>
  create(std::pair<uint32_t, uint32_t> size_) {
    return std::make_shared<texture2d<Color>>(size_);
  }

  friend bool operator==(const texture2d &lhs, const texture2d &rhs) {
    return std::equal(lhs.data.get(), lhs.data.get() + lhs.bytes(),
                      rhs.data.get(), rhs.data.get() + rhs.bytes());
  }

  friend bool operator!=(const texture2d &lhs, const texture2d &rhs) {
    return !(lhs == rhs);
  }

  inline uint8_t *raw_data() { return data.get(); }

  inline const uint8_t *raw_data() const { return data.get(); }

  static texture_type get_type_for_string(std::string const &type_string) {
    if (type_string == "uint8")
      return TEXTURE_TYPE_UINT8;
    else if (type_string == "uint16")
      return TEXTURE_TYPE_UINT16;
    else if (type_string == "float")
      return TEXTURE_TYPE_FLOAT;
    else if (type_string == "double")
      return TEXTURE_TYPE_DOUBLE;

    return TEXTURE_TYPE_UNKNOWN;
  }

  explicit operator bool() const { return valid(); }

  inline bool valid() const {
    return (size.first != 0 || size.second != 0) && data.get() != nullptr;
  }

  inline bool contains(int x, int y) const {
    return 0 <= x && x < size.first && 0 <= y && y < size.second;
  }

  void crop(std::pair<uint32_t, uint32_t> off_,
            std::pair<uint32_t, uint32_t> size_) {
    assert((off_.first + size_.first) <= size.first &&
           (off_.second + size_.second) <= size.second);

    auto stride_ = size_.first * color_size;
    auto data_ = std::make_unique<uint8_t[]>(stride_ * size_.second);

    for (size_t y = 0; y < size_.second; ++y) {
      std::copy((this->color_begin(y + off_.second) + off_.first),
                (this->color_begin(y + off_.second) + off_.first + size_.first),
                reinterpret_cast<Color *>(data_.get() + stride_ * y));
    }

    size = size_;
    data = std::move(data_);
  }

  std::shared_ptr<texture2d<Color>> duplicate() const {
    auto copy_ = std::make_shared<texture2d<Color>>(size);
    std::copy(data.get(), data.get() + bytes(), copy_->data.get());
    return copy_;
  }

  std::shared_ptr<texture2d<Color>>
  duplicate(std::pair<uint32_t, uint32_t> off_,
            std::pair<uint32_t, uint32_t> size_) const {
    assert((off_.first + size_.first) <= size.first &&
           (off_.second + size_.second) <= size.second);

    auto copy_ = std::make_shared<texture2d<Color>>(size_);

    for (uint32_t y = 0; y < size_.second; y++) {
      const size_t srcOffset =
          (y + off_.second) * stride() + off_.first * color_size;
      const size_t dstOffset = y * copy_->stride();
      ::memcpy(copy_->data.get() + dstOffset, data.get() + srcOffset,
               size_.first * color_size);
    }
    return copy_;
  }

  template <typename T = texture2d<Color>> T clone() const {
    T copy_(size);
    std::copy(data.get(), data.get() + bytes(), copy_.data.get());
    return copy_;
  }

  void fill_color(const Color &color) {
    std::fill(color_begin(), color_end(), color);
  }

  void fill_pixel(pixel_type value) {
    std::fill(pixel_begin(), pixel_end(), value);
  }

  void fill(uint8_t value) {
    std::fill(data.get(), data.get() + bytes(), value);
  }

  void resize(std::pair<uint32_t, uint32_t> size_) {
    if (size.first == size_.first && size.second == size_.second) {
      return;
    }
    texture2d<Color> newimage(size_);
    newimage.fill(0);
    copy(*this, newimage, {0, 0}, {0, 0},
         {std::min(size.first, size_.first),
          std::min(size.second, size_.second)});
    operator=(std::move(newimage));
  }

  inline size_t color_count() const { return size.first * size.second; }
  inline size_t pixel_count() const { return color_count() * channels_; }

  inline iterator begin() noexcept {
    return impl::texture_row_iterator<Color>(impl::texture_row<Color>(this, 0));
  }
  inline const_iterator begin() const noexcept {
    return impl::const_texture_row_iterator<Color>(
        impl::const_texture_row<Color>(this, 0));
  }
  inline const_iterator cbegin() const noexcept {
    return impl::const_texture_row_iterator<Color>(
        impl::const_texture_row<Color>(this, 0));
  }

  inline iterator end() noexcept {
    return impl::texture_row_iterator<Color>(
        impl::texture_row<Color>(this, this->size.second));
  }
  inline const_iterator end() const noexcept {
    return impl::const_texture_row_iterator<Color>(
        impl::const_texture_row<Color>(this, this->size.second));
  }
  inline const_iterator cend() const noexcept {
    return impl::const_texture_row_iterator<Color>(
        impl::const_texture_row<Color>(this, this->size.second));
  }

  inline Color *color_begin() noexcept { return color_data(); }
  inline Color const *color_begin() const noexcept { return color_data(); }
  inline Color *color_end() noexcept { return color_data() + color_count(); }
  inline Color const *color_end() const noexcept {
    return color_data() + color_count();
  }

  inline Color *color_begin(size_t r) noexcept {
    return (color_data() + r * stride());
  }
  inline Color const *color_begin(size_t r) const noexcept {
    return (color_data() + r * stride());
  }
  inline Color *color_end(size_t r) noexcept {
    return (color_begin(r) + stride());
  }
  inline Color const *color_end(size_t r) const noexcept {
    return (color_begin(r) + stride());
  }

  RowPointersType row_pointers() {
    using size_type = typename RowPointersType::size_type;
    RowPointersType row_pointers(this->height());

    for (size_t y = 0; y < this->height(); ++y) {
      row_pointers[y] = reinterpret_cast<uint8_t *>(this->color_begin(y));
    }

    return row_pointers;
  }

  ConstRowPointersType const_row_pointers() {
    using size_type = typename ConstRowPointersType::size_type;
    ConstRowPointersType row_pointers(this->height());

    for (size_t y = 0; y < this->height(); ++y) {
      row_pointers[y] = reinterpret_cast<const uint8_t *>(this->color_begin(y));
    }

    return row_pointers;
  }

  inline pixel_type *pixel_begin() noexcept { return pixel_data(); }
  inline pixel_type const *pixel_begin() const noexcept { return pixel_data(); }
  inline pixel_type *pixel_end() noexcept {
    return pixel_data() + pixel_count();
  }
  inline pixel_type const *pixel_end() const noexcept {
    return pixel_data() + pixel_count();
  }

  inline Color const &at(size_t index) const { return color_data()[index]; }
  inline Color const &at(size_t x, size_t y) const {
    assert(x <= size.first && y <= size.second);
    return *reinterpret_cast<const Color *>(raw_data() +
                                            (y * stride() + x * color_size));
  }
  inline pixel_type at(size_t x, size_t y, size_t c) const {
    return pixel_get(at(x, y), c);
  }

  inline Color &at(size_t index) { return color_data()[index]; }
  inline Color &at(size_t x, size_t y) {
    assert(x <= size.first && y <= size.second);
    return *reinterpret_cast<Color *>(raw_data() +
                                      (y * stride() + x * color_size));
  }
  inline pixel_type &at(size_t x, size_t y, size_t c) {
    return pixel_get(at(x, y), c);
  }

  inline Color &operator[](size_t index) { return color_data()[index]; }
  inline Color const &operator[](size_t index) const {
    return color_data()[index];
  }

  inline Color &operator()(size_t index) { return color_data()[index]; }
  inline Color const &operator()(size_t index) const {
    return color_data()[index];
  }

  inline Color &operator()(size_t x, size_t y) {
    assert(x <= size.first && y <= size.second);
    return *reinterpret_cast<Color *>(raw_data() +
                                      (y * stride() + x * color_size));
  }
  inline Color const &operator()(size_t x, size_t y) const {
    assert(x <= size.first && y <= size.second);
    return *reinterpret_cast<const Color *>(raw_data() +
                                            (y * stride() + x * color_size));
  }

  inline pixel_type &operator()(size_t x, size_t y, size_t c) {
    return pixel_get(at(x, y), c);
  }
  inline pixel_type operator()(size_t x, size_t y, size_t c) const {
    return pixel_get(at(x, y), c);
  }

  inline pixel_type *pixel_data() {
    return reinterpret_cast<pixel_type *>(raw_data());
  }
  inline const pixel_type *pixel_data() const {
    return reinterpret_cast<const pixel_type *>(raw_data());
  }

  inline Color *color_data() { return reinterpret_cast<Color *>(raw_data()); }
  inline const Color *color_data() const {
    return reinterpret_cast<const Color *>(raw_data());
  }

  inline Color linear_color(openvdb::Vec2f i) const {
    return linear_color(i.x(), i.y());
  }

  Color linear_color(float x, float y) const {
    x = std::max(0.0f, std::min(static_cast<float>(size.first - 1), x));
    y = std::max(0.0f, std::min(static_cast<float>(size.second - 1), y));

    size_t const floor_x = static_cast<size_t>(x);
    size_t const floor_y = static_cast<size_t>(y);
    size_t const floor_xp1 =
        std::min<float>(static_cast<float>(floor_x + 1), size.first - 1);
    size_t const floor_yp1 =
        std::min<float>(static_cast<float>(floor_y + 1), size.second - 1);

    float const w1 = x - static_cast<float>(floor_x);
    float const w0 = 1.0f - w1;
    float const w3 = y - static_cast<float>(floor_y);
    float const w2 = 1.0f - w3;

    return interpolate(
        this->color(floor_x, floor_y), this->color(floor_x, floor_yp1),
        this->color(floor_xp1, floor_y), this->color(floor_xp1, floor_yp1),
        openvdb::Vec4d{w0 * w2, w1 * w2, w0 * w3, w1 * w3});
  }

  inline Color &color(std::pair<uint32_t, uint32_t> i) {
    return *reinterpret_cast<Color *>(
        raw_data() + (i.second * stride() + i.first * color_size));
  }

  inline const Color &color(std::pair<uint32_t, uint32_t> i) const {
    return *reinterpret_cast<const Color *>(
        raw_data() + (i.second * stride() + i.first * color_size));
  }

  inline pixel_type &pixel(std::pair<uint32_t, uint32_t> i, size_t c) {
    return pixel_get(color(i), c);
  }

  inline pixel_type pixel(std::pair<uint32_t, uint32_t> i, size_t c) const {
    return pixel_get(color(i), c);
  }

  inline Color &color(size_t x, size_t y) {
    assert(x <= size.first && y <= size.second);
    return *reinterpret_cast<Color *>(raw_data() +
                                      (y * stride() + x * color_size));
  }

  inline const Color &color(size_t x, size_t y) const {
    assert(x <= size.first && y <= size.second);
    return *reinterpret_cast<const Color *>(raw_data() +
                                            (y * stride() + x * color_size));
  }

  inline pixel_type &pixel(size_t x, size_t y, size_t c) {
    return pixel_get(color(x, y), c);
  }

  inline pixel_type pixel(size_t x, size_t y, size_t c) const {
    return pixel_get(color(x, y), c);
  }

  inline size_t channels() const { return channels_; }
  inline size_t dim() const { return 2; }
  inline size_t stride() const { return color_size * size.first; }
  inline size_t bytes() const { return stride() * size.second; }

  inline uint64_t width() const { return size.first; }
  inline uint64_t height() const { return size.second; }

  static void clear(texture2d<Color> &dstImg,
                    const std::pair<uint32_t, uint32_t> &pt,
                    const std::pair<uint32_t, uint32_t> &size) {
    if (size.first == 0 && size.second == 0) {
      return;
    }

    if (!dstImg.valid()) {
      throw std::invalid_argument("invalid destination for image clear");
    }

    if (size.first > dstImg.size.first || size.second > dstImg.size.second ||
        pt.first > dstImg.size.first - size.first ||
        pt.second > dstImg.size.second - size.second) {
      throw std::out_of_range(
          "out of range destination coordinates for image clear");
    }

    uint8_t *dstData = dstImg.data.get();

    for (uint32_t y = 0; y < size.second; y++) {
      const size_t dstOffset =
          (pt.second + y) * dstImg.stride() + pt.first * color_size;
      std::memset(dstData + dstOffset, 0, size.first * color_size);
    }
  }

  static void copy(const texture2d<Color> &srcImg, texture2d<Color> &dstImg,
                   const std::pair<uint32_t, uint32_t> &srcPt,
                   const std::pair<uint32_t, uint32_t> &dstPt,
                   const std::pair<uint32_t, uint32_t> &size) {
    if (size.first == 0 && size.second == 0) {
      return;
    }

    if (!srcImg.valid()) {
      throw std::invalid_argument("invalid source for image copy");
    }

    if (!dstImg.valid()) {
      throw std::invalid_argument("invalid destination for image copy");
    }

    if (size.first > srcImg.size.first || size.second > srcImg.size.second ||
        srcPt.first > srcImg.size.first - size.first ||
        srcPt.second > srcImg.size.second - size.second) {
      throw std::out_of_range("out of range source coordinates for image copy");
    }

    if ((dstPt.first + size.first) > dstImg.size.first ||
        (dstPt.second + size.second) > dstImg.size.second) {
      throw std::out_of_range(
          "out of range destination coordinates for image copy");
    }

    const uint8_t *srcData = srcImg.data.get();
    uint8_t *dstData = dstImg.data.get();

    assert(srcData != dstData);

    for (uint32_t y = 0; y < size.second; y++) {
      const size_t srcOffset =
          (srcPt.second + y) * srcImg.stride() + srcPt.first * color_size;
      const size_t dstOffset =
          (dstPt.second + y) * dstImg.stride() + dstPt.first * color_size;
      ::memcpy(dstData + dstOffset, srcData + srcOffset,
               size.first * color_size);
    }
  }
};

} // namespace flywave

namespace std {

template <typename Color> struct hash<::flywave::texture2d<Color>> {
  size_t operator()(const ::flywave::texture2d<Color> &obj) const { return -1; }
};

} // namespace std
