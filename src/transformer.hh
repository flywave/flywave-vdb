#pragma once

#include <openvdb/Types.h>

namespace flywave {

class grid_transform {
public:
  virtual ~grid_transform() = default;

  virtual bool isAffine() const = 0;
  virtual bool isIdentity() const = 0;

  virtual openvdb::Vec3d transform(const openvdb::Vec3d &) const = 0;
  virtual openvdb::Vec3d invTransform(const openvdb::Vec3d &) const = 0;
};

struct matrix_transform : public grid_transform {
  matrix_transform()
      : mat(openvdb::Mat4R::identity()), invMat(openvdb::Mat4R::identity()) {}
  matrix_transform(const openvdb::Mat4R &xform)
      : mat(xform), invMat(xform.inverse()) {}

  bool isAffine() const { return openvdb::math::isAffine(mat); }
  bool isIdentity() const { return openvdb::math::isIdentity(mat); }

  openvdb::Vec3R transform(const openvdb::Vec3R &pos) const {
    return mat.transformH(pos);
  }

  openvdb::Vec3R invTransform(const openvdb::Vec3R &pos) const {
    return invMat.transformH(pos);
  }

  openvdb::Mat4R mat, invMat;
};
} // namespace flywave
