#pragma once

#include <cmath>
#include <stdio.h>
#include <string.h>

#include <openvdb/Types.h>

namespace flywave {
namespace voxelize {
class pixel;
} // namespace voxelize
} // namespace flywave

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace math {

inline bool isApproxEqual(const flywave::voxelize::pixel &a,
                          const flywave::voxelize::pixel &b);

inline bool isApproxEqual(const flywave::voxelize::pixel &a,
                          const flywave::voxelize::pixel &b,
                          const flywave::voxelize::pixel &);

} // namespace math
} // namespace openvdb

namespace flywave {
namespace voxelize {

namespace vdb = openvdb::v8_1;

class pixel_data {
public:
  enum class type_t : uint8_t { color, material, material_and_color, invalid };

  pixel_data() : _type(type_t::invalid) {}

  pixel_data(uint8_t material, const vdb::math::Vec4<uint8_t> &color,
             uint16_t feature_id = -1)
      : _type(type_t::material_and_color), _material_id(material),
        _feature_id(feature_id), _color(color) {}

  pixel_data(const vdb::math::Vec4<uint8_t> &color, uint16_t feature_id = -1)
      : _type(type_t::color), _material_id(0), _color(color) {}

  pixel_data(uint8_t material, uint16_t feature_id = -1)
      : _type(type_t::material) {
    _material_id = material;
  }

  bool has_color() const {
    return _type == type_t::color || _type == type_t::material_and_color;
  }

  type_t _type;
  uint8_t _material_id = 0;
  uint16_t _feature_id = 0;
  vdb::math::Vec4<uint8_t> _color;
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
} // namespace flywave

namespace openvdb {
namespace OPENVDB_VERSION_NAME {
namespace math {

inline bool isApproxEqual(const flywave::voxelize::pixel &a,
                          const flywave::voxelize::pixel &b) {
  return isApproxEqual(a._value, b._value);
}

inline bool isApproxEqual(const flywave::voxelize::pixel &a,
                          const flywave::voxelize::pixel &b,
                          const flywave::voxelize::pixel &t) {
  return isApproxEqual(a._value, b._value, t._value);
}

inline flywave::voxelize::pixel abs(const flywave::voxelize::pixel &v) {
  return flywave::voxelize::pixel(std::abs(v._value));
}
} // namespace math
} // namespace OPENVDB_VERSION_NAME

namespace OPENVDB_VERSION_NAME {
template <>
inline flywave::voxelize::pixel zeroVal<flywave::voxelize::pixel>() {
  return flywave::voxelize::pixel(0);
}
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb
