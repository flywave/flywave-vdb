#pragma once

#include <openvdb/Types.h>

#include "zero.hh"

namespace flywave {

template <typename T> struct tolerance {
  operator T() const { return zero<T>(); }
};

template <> struct tolerance<openvdb::math::half> {
  operator openvdb::math::half() const { return {1e-6f}; }
};

template <> struct tolerance<float> {
  operator float() const { return 1e-8f; }
};

template <> struct tolerance<double> {
  operator double() const { return 1e-15; }
};
} // namespace flywave
