#pragma once

#include "barycentric.hh"
#include "types.hh"

#include <memory>

namespace flywave {
namespace voxelize {

class st_policy {
public:
  virtual void start_triangle(const face_index_t &index,
                              const triangle3<double> &tri) = 0;

  virtual Eigen::Matrix<double, 2, 1>
  eval_uv(const Eigen::Matrix<double, 3, 1> &point) = 0;

  virtual std::unique_ptr<st_policy> make_shared() = 0;
};

class uv_coord_reader {
public:
  virtual triangle2<double> get(face_index_t index) = 0;
};

class uv_st_policy : public st_policy {
public:
  uv_st_policy(std::unique_ptr<uv_coord_reader> uvs) : _uvs(std::move(uvs)) {}

  void start_triangle(const face_index_t &index,
                      const triangle3<double> &tri) override {
    if (_uvs) {
      _convert = bary_convert(tri, _uvs->get(index));
    } else {
      _convert = bary_convert(tri, _uvptr->get(index));
    }
  }

  Eigen::Matrix<double, 2, 1>
  eval_uv(const Eigen::Matrix<double, 3, 1> &point) override {
    return _convert.bary2uv(_convert.pos2bary(point));
  }

  std::unique_ptr<st_policy> make_shared() override {
    return std::make_unique<uv_st_policy>(uv_st_policy(*_uvs));
  }

private:
  uv_st_policy(uv_coord_reader &uvs) : _uvptr(&uvs) {}

private:
  std::unique_ptr<uv_coord_reader> _uvs;
  uv_coord_reader *_uvptr = nullptr;
  bary_convert _convert;
};

class only_vertex_policy : public st_policy {
public:
  only_vertex_policy(const Eigen::Matrix<float, 3, 1> up,
                     const bbox2<float> &box)
      : _bbox(box), _up(up) {}

  inline void start_triangle(const face_index_t &index,
                             const triangle3<double> &tri) override {}

  Eigen::Matrix<double, 2, 1>
  eval_uv(const Eigen::Matrix<double, 3, 1> &point) override {
    return Eigen::Matrix<double, 2, 1>(0, 0);
  }

public:
  bbox2<float> _bbox;
  Eigen::Matrix<float, 3, 1> _up;
};

} // namespace voxelize
} // namespace flywave
