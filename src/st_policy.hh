#pragma once

#include "barycentric.hh"
#include "types.hh"

#include <memory>

namespace flywave {

class st_policy {
public:
  virtual ~st_policy() = default;

  virtual void start_triangle(const face_index_t &index,
                              const triangle3<double> &tri) = 0;

  virtual openvdb::Vec2d eval_uv(const openvdb::Vec3d &point) = 0;

  virtual std::unique_ptr<st_policy> make_shared() = 0;
};

class uv_coord_reader {
public:
  virtual ~uv_coord_reader() = default;

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

  openvdb::Vec2d eval_uv(const openvdb::Vec3d &point) override {
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
  only_vertex_policy(const openvdb::Vec3d up, const bbox2<float> &box)
      : _bbox(box), _up(up) {}

  inline void start_triangle(const face_index_t &index,
                             const triangle3<double> &tri) override {}

  openvdb::Vec2d eval_uv(const openvdb::Vec3d &point) override {
    return openvdb::Vec2d(0, 0);
  }

public:
  bbox2<float> _bbox;
  openvdb::Vec3d _up;
};

} // namespace flywave
