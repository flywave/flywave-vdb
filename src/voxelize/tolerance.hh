#pragma once

#include <openvdb/Types.h>

#include "zero.hh"

namespace flywave {

namespace vdb = openvdb::OPENVDB_VERSION_NAME;

template <typename T> struct tolerance {
  operator T() const { return zero<T>(); }
};

template <> struct tolerance<vdb::math::half> {
  operator vdb::math::half() const { return {1e-6f}; }
};

template <> struct tolerance<float> {
  operator float() const { return 1e-8f; }
};

template <> struct tolerance<double> {
  operator double() const { return 1e-15; }
};
} // namespace flywave
