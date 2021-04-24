#pragma once
#include <cmath>
#include <stdio.h>
#include <string.h>

namespace flywave {
namespace voxelize {
class pixel;
}

namespace math {

inline constexpr bool is_approx_equal(const flywave::voxelize::pixel &a,
                                      const flywave::voxelize::pixel &b);

inline constexpr bool is_approx_equal(const voxelize::pixel &a,
                                      const voxelize::pixel &b,
                                      const voxelize::pixel &);

} // namespace math
} // namespace flywave

#include <flywave/math/color.hh>
#include <flywave/math/vector_lib.hh>
#include <flywave/math/zero.hh>
namespace flywave {
namespace voxelize {

class pixel_data {
public:
  enum class type_t : uint8_t { color, material, material_and_color, invalid };

  pixel_data() : _type(type_t::invalid) {}

  pixel_data(uint8_t material, const color4<uint8_t> &color,
             uint16_t feature_id = -1)
      : _type(type_t::material_and_color), _material_id(material),
        _color(color), _feature_id(feature_id) {}

  pixel_data(const color4<uint8_t> &color, uint16_t feature_id = -1)
      : _type(type_t::color), _material_id(0), _color(color) {}

  pixel_data(uint8_t material, uint16_t feature_id = -1)
      : _type(type_t::material) {
    _material_id = material;
  }

  template <typename Describer> auto describe_type(Describer f) {
    return f(_type, _color, _material_id, _feature_id);
  }

  bool has_color() const {
    return _type == type_t::color || _type == type_t::material_and_color;
  }

  type_t _type;
  uint8_t _material_id = 0;
  uint16_t _feature_id = 0;
  color4<uint8_t> _color;
};

class pixel {
public:
  pixel() : _value(0) {}

  pixel(float value) : _value(value) {}

  pixel(const pixel &pix) noexcept { memcpy(this, &pix, sizeof(pixel)); }

  pixel(const pixel_data &data) : _data(data) {}

  const pixel &operator=(const pixel &pix) {
    memcpy(this, &pix, sizeof(pixel));
    return *this;
  }

  constexpr bool operator<(const pixel &t) const { return _value < t._value; }
  constexpr bool operator>(const pixel &t) const { return _value > t._value; }

  constexpr bool operator<(const float &t) const { return _value < t; }
  constexpr bool operator>(const float &t) const { return _value > t; }

  constexpr bool operator<(const double &t) const { return _value < t; }
  constexpr bool operator>(const double &t) const { return _value > t; }

  constexpr bool operator<(const int &t) const { return _value < t; }
  constexpr bool operator>(const int &t) const { return _value > t; }

  const pixel &operator+=(const double &t) {
    _value += t;
    return *this;
  }
  const pixel &operator+=(const float &t) {
    _value += t;
    return *this;
  }
  const pixel &operator+=(const pixel &t) {
    _value += t._value;
    return *this;
  }

  constexpr bool is_equal(const pixel &e) const {
    return _value == e._value; //&& e._data == _data;
  }

  constexpr bool is_greater(const pixel &e) const { return _value > e._value; }

  constexpr bool is_less(const pixel &e) const { return _value < e._value; }

  template <class T> pixel operator+(const T &t) const {
    return pixel(t + _value);
  }

  template <class T> pixel operator-(const T &t) const {
    return pixel(_value - t);
  }
  pixel operator-() const { return pixel(-_value); }

  bool operator==(const pixel &rhs) const { return _value == rhs._value; }

  bool operator!=(const pixel &rhs) const { return !((*this) == rhs); }

  template <typename Describer> auto describe_type(Describer f) {
    return f(_data);
  }

  operator float() const { return _value; }

  union {
    float _value;
    pixel_data _data;
  };
};

template <typename T,
          typename = std::enable_if_t<std::is_arithmetic<T>::value, T>>
inline bool operator==(const T &t, const pixel &c) {
  return t == c._value;
}

template <typename T,
          typename = std::enable_if_t<std::is_arithmetic<T>::value, T>>
inline bool operator==(const pixel &c, const T &t) {
  return t == c._value;
}

template <typename T,
          typename = std::enable_if_t<std::is_arithmetic<T>::value, T>>
inline bool operator<(const T &t, const pixel &c) {
  return t < c._value;
}

template <typename T,
          typename = std::enable_if_t<std::is_arithmetic<T>::value, T>>
inline bool operator<=(const T &t, const pixel &c) {
  return t <= c._value;
}

template <typename T,
          typename = std::enable_if_t<std::is_arithmetic<T>::value, T>>
inline bool operator>(const T &t, const pixel &c) {
  return t > c._value;
}

template <typename T,
          typename = std::enable_if_t<std::is_arithmetic<T>::value, T>>
inline bool operator>(const pixel &c, const T &t) {
  return c._value > t;
}

template <typename T,
          typename = std::enable_if_t<std::is_arithmetic<T>::value, T>>
inline bool operator>=(const T &t, const pixel &c) {
  return t >= c._value;
}

inline std::ostream &operator<<(std::ostream &os, const pixel &s) { return os; }

} // namespace voxelize

namespace math {

inline constexpr bool is_approx_equal(const flywave::voxelize::pixel &a,
                                      const flywave::voxelize::pixel &b) {
  return is_approx_equal(a._value, b._value);
}

inline constexpr bool is_approx_equal(const voxelize::pixel &a,
                                      const voxelize::pixel &b,
                                      const voxelize::pixel &t) {
  return is_approx_equal(a._value, b._value, t._value);
}

inline flywave::voxelize::pixel abs(const flywave::voxelize::pixel &v) {
  // auto pix = v;
  // pix._value = std::abs(v._value);
  return flywave::voxelize::pixel(std::abs(v._value));
}

} // namespace math

template <> struct zero<voxelize::pixel> {
  operator voxelize::pixel() const { return voxelize::pixel(0); }
};
} // namespace flywave

namespace std {

inline float abs(const flywave::voxelize::pixel &v) {
  return std::abs(v._value);
}

inline float max(const flywave::voxelize::pixel &v,
                 const flywave::voxelize::pixel &v2) {
  return std::max(v._value, v._value);
}

template <> class std::numeric_limits<flywave::voxelize::pixel> {
public:
  static constexpr bool is_specialized = false;
  static flywave::voxelize::pixel min() noexcept {
    return std::numeric_limits<float>::min();
  }
  static flywave::voxelize::pixel max() noexcept {
    return std::numeric_limits<float>::max();
  }
  static flywave::voxelize::pixel lowest() noexcept {
    return std::numeric_limits<float>::lowest();
  }

  static constexpr int digits = 0;
  static constexpr int digits10 = 0;
  static constexpr bool is_signed = true;
  static constexpr bool is_integer = false;
  static constexpr bool is_exact = false;
  static constexpr int radix = 0;
  static flywave::voxelize::pixel epsilon() noexcept {
    return std::numeric_limits<float>::epsilon();
  }
  static flywave::voxelize::pixel round_error() noexcept {
    return std::numeric_limits<float>::round_error();
  }

  static constexpr int min_exponent = 0;
  static constexpr int min_exponent10 = 0;
  static constexpr int max_exponent = 0;
  static constexpr int max_exponent10 = 0;

  static constexpr bool has_infinity = true;
  static constexpr bool has_quiet_NaN = true;
  static constexpr bool has_signaling_NaN = false;
  static constexpr float_denorm_style has_denorm = denorm_absent;
  static constexpr bool has_denorm_loss = false;
  static flywave::voxelize::pixel infinity() noexcept {
    return std::numeric_limits<float>::infinity();
  }
  static flywave::voxelize::pixel quiet_NaN() noexcept {
    return std::numeric_limits<float>::quiet_NaN();
  }
  static flywave::voxelize::pixel signaling_NaN() noexcept {
    return std::numeric_limits<float>::signaling_NaN();
  }
  static flywave::voxelize::pixel denorm_min() noexcept {
    return std::numeric_limits<float>::denorm_min();
  }

  static constexpr bool is_iec559 = false;
  static constexpr bool is_bounded = false;
  static constexpr bool is_modulo = false;

  static constexpr bool traps = false;
  static constexpr bool tinyness_before = false;
  static constexpr float_round_style round_style = round_toward_zero;
};

template <> struct is_signed<flywave::voxelize::pixel> : std::true_type {};

template <> struct is_scalar<flywave::voxelize::pixel> : std::true_type {};

} // namespace std
