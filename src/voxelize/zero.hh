#pragma once

#include <openvdb/Types.h>

namespace flywave {

template <typename V> struct zero {
  operator V() const { return 0; }
};

template <> struct zero<bool> {
  operator bool() const { return false; }
};

template <> struct zero<openvdb::math::half> {
  operator openvdb::math::half() const {
    return openvdb::math::half();
  }
};

template <> struct zero<float> {
  operator float() const { return 0.f; }
};

template <> struct zero<double> {
  operator double() const { return double(0.0); }
};
} // namespace flywave
