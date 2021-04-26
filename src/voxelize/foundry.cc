#pragma once

#include "foundry.hh"
#include "index.hh"
#include "projection.hh"
#include "resolution.hh"
#include "sampler.hh"
#include "voxel_pot.hh"

#include "tolerance.hh"

#include <openvdb/math/Transform.h>
#include <openvdb/tools/VolumeToMesh.h>

#include <wrap/io_trimesh/export_obj.h>

namespace flywave {
namespace voxelize {

class textute_foundry::impl {
public:
  using sampler_type = sampling_result<pixel_grid::ValueType>;

public:
  impl(vertex_grid::Ptr cgrid, pixel_grid::Ptr pgrid) {
    _query = make_near_voxels_index<vertex_grid, pixel_grid>(cgrid, pgrid);
  }

  std::vector<sampler_type> extract(std::vector<openvdb::Vec3d> coords) const {
    return _query->extract(std::move(coords));
  }

  static texture2d<vdb::math::Vec4<uint8_t>>::Ptr
  project_to_image(std::vector<sampler_type> points,
                   const triangle_projection &prj,
                   texture2d<vdb::math::Vec4<uint8_t>>::Ptr);

  size_t pixel_size() const { return _query->pixel_count(); }

private:
  std::unique_ptr<triangle_range_query<pixel_grid>> _query;
};

textute_foundry::textute_foundry(vertex_grid::Ptr cgrid, pixel_grid::Ptr pgrid,
                                 openvdb::Mat4d mat, float pixel_pad)
    : _query(std::make_unique<impl>(cgrid, pgrid)), _grid(cgrid), _mat(mat),
      _pixel_pad(pixel_pad) {}

texture2d<vdb::math::Vec4<uint8_t>>::Ptr textute_foundry::extract(
    const fmesh_tri_patch &tri,
    texture2d<vdb::math::Vec4<uint8_t>>::Ptr texture) const {
  fmesh_tri_patch ptri = tri;
  ptri.p1 = _grid->transform().worldToIndex(tri.p1);
  ptri.p2 = _grid->transform().worldToIndex(tri.p2);
  ptri.p3 = _grid->transform().worldToIndex(tri.p3);

  triangle_projection proj(ptri, _pixel_pad);

  std::vector<vector2<int>> outcoordl;
  auto q = proj.to_voxels(outcoordl);
  return impl::project_to_image(_query->extract(std::move(q)), proj,
                                std::move(outcoordl), texture);
}

size_t textute_foundry::pixel_size() const { return _query->pixel_size(); }

textute_foundry::~textute_foundry() = default;

texture2d<vdb::math::Vec4<uint8_t>>::Ptr
textute_foundry::impl::project_to_image(
    std::vector<sampler_type> points, const triangle_projection &prj,
    texture2d<vdb::math::Vec4<uint8_t>>::Ptr img) {

  auto h = img->height();

  for (auto &it : points) {
    auto uv = prj.point_to_uv(it._tri_coord);
    if (uv.x() >= 0 && uv.y() >= 0) {

      uint64_t min_x = std::min(uint64_t(std::floor(uv.x())), img->width() - 1);
      uint64_t min_y = std::min(uint64_t(std::floor(h - uv.y() - 1)), h - 1);
      uint64_t max_x = std::min(uint64_t(std::ceil(uv.x())), img->width() - 1);
      uint64_t max_y = std::min(uint64_t(std::ceil(h - uv.y() - 1)), h - 1);

      img->color(min_x, min_y) = it._value._data._color;
      img->color(max_x, max_y) = it._value._data._color;
      img->color(min_x, max_y) = it._value._data._color;
      img->color(max_x, min_y) = it._value._data._color;
    }
  }
  return img;
}

void triangle_foundry::make_mesh_mark_seam(std::vector<vertext_type> &points,
                                           std::vector<triangle_type> &tri,
                                           std::vector<quad_type> &quads,
                                           double isovalue, double adapter) {
  auto ptr = make_shared<out_setting_volume_mesh_func>();
  vdb::tools::volume_to_mesh(*_grid, points, tri, quads, isovalue, ptr,
                             adapter);
}

void make_io_triangles(lm::virtual_array<lm::io_triangle> &rettriangles,
                       voxel_pot &pot, const matrix44<double> &mat,
                       size_t text_offset, size_t mtl_offset,
                       shared_ptr<border_lock> lock,
                       shared_ptr<filter_triangle> filter, double fquality,
                       double isovalue, double adapter) {
  shared_ptr<triangle_foundry> vfoundry;

  vfoundry = make_shared<triangle_foundry>(pot.voxel_grid());

  std::unique_ptr<std::vector<vertext_type>> points_ptr =
      std::make_unique<std::vector<vertext_type>>();
  std::unique_ptr<std::vector<triangle_type>> triangles_ptr =
      std::make_unique<std::vector<triangle_type>>();
  std::unique_ptr<std::vector<quad_type>> quads_ptr =
      std::make_unique<std::vector<quad_type>>();

  auto &points = *points_ptr;
  auto &triangles = *triangles_ptr;
  auto &quads = *quads_ptr;

  vfoundry->make_mesh_mark_seam(points, triangles, quads, isovalue, adapter)
      .get();

  std::unique_ptr<std::unordered_map<int, std::vector<lm::io_triangle>>>
      group_triangles_ptr = std::make_unique<
          std::unordered_map<int, std::vector<lm::io_triangle>>>();

  auto &group_triangles = *group_triangles_ptr;
  size_t count = 0;
  auto accessor = pot.pixel_grid()->get_accessor();

  auto add_triangles = [&count, &points, &pot, &filter, &group_triangles,
                        text_offset, mtl_offset,
                        &accessor](triangle_type &tri) {
    auto &v0 = points[tri[0]];
    auto &v1 = points[tri[1]];
    auto &v2 = points[tri[2]];

    if (filter && !filter->valid(v0, v1, v2)) {
      return;
    }

    struct mtsetting {
      int tex;
      int mtl;
      globe_feature_id_t _globe_feature_id;
      bool b;

      operator bool() const { return b; }
    };

    auto get_mt_config = [&](const vertext_type &p) {
      vector3<float> pt = pot.voxel_resolution().world_to_index(p);

      vdb::coord ijk(pt[0], pt[1], pt[2]);
      mtsetting mtconfig;

      pixel pix;
      if (!seach_vertex_value(accessor, ijk, pix)) {
        mtconfig.tex = -1;
        mtconfig.mtl = pix._data._material_id;
      } else {
        if (pix._data._type == pixel_data::type_t::material) {
          mtconfig.tex = -1;
          mtconfig.mtl = pix._data._material_id;
        } else if (pix._data._type != pixel_data::type_t::invalid) {
          mtconfig.tex = 0;
          mtconfig.mtl = pix._data._material_id;
        } else {
          mtconfig.tex = -1;
          mtconfig.mtl = pix._data._material_id;
        }
      }

      return mtconfig;
    };

    lm::io_triangle io_tri;

    io_tri.node = 0;
    {
      mtsetting vp0 = get_mt_config(v0);
      mtsetting vp1 = get_mt_config(v1);
      mtsetting vp2 = get_mt_config(v2);

      if (vp0 && vp0.tex == 0) {
        io_tri.tex = vp0.tex;
        io_tri.mtl = vp0.mtl;
        io_tri.feature_id = vp0._globe_feature_id;
      } else if (vp1 && vp1.tex == 0) {
        io_tri.tex = vp1.tex;
        io_tri.mtl = vp1.mtl;
        io_tri.feature_id = vp1._globe_feature_id;
      } else if (vp2 && vp2.tex == 0) {
        io_tri.tex = vp2.tex;
        io_tri.mtl = vp2.mtl;
        io_tri.feature_id = vp2._globe_feature_id;
      } else {
        if (vp0) {
          io_tri.tex = vp0.tex;
          io_tri.mtl = vp0.mtl;
          io_tri.feature_id = vp0._globe_feature_id;
        } else if (vp1) {
          io_tri.tex = vp0.tex;
          io_tri.mtl = vp0.mtl;
          io_tri.feature_id = vp0._globe_feature_id;
        } else if (vp2) {
          io_tri.tex = vp0.tex;
          io_tri.mtl = vp0.mtl;
          io_tri.feature_id = vp0._globe_feature_id;
        } else {
          return;
          io_tri.tex = -1;
          io_tri.mtl = -1;
          io_tri.feature_id = -1;
        }
      }
    }

    lm::io_vertex _v0{};
    lm::io_vertex _v1{};
    lm::io_vertex _v2{};

    _v0.v[0] = v0[0];
    _v0.v[1] = v0[1];
    _v0.v[2] = v0[2];

    _v0.t[0] = 0.0f;
    _v0.t[1] = 0.0f;

    _v1.v[0] = v1[0];
    _v1.v[1] = v1[1];
    _v1.v[2] = v1[2];

    _v1.t[0] = 0.0f;
    _v1.t[1] = 0.0f;

    _v2.v[0] = v2[0];
    _v2.v[1] = v2[1];
    _v2.v[2] = v2[2];

    _v2.t[0] = 0.0f;
    _v2.t[1] = 0.0f;

    io_tri.vertices[0] = _v2;
    io_tri.vertices[1] = _v1;
    io_tri.vertices[2] = _v0;

    auto iter = group_triangles.find(io_tri.mtl);
    if (iter == group_triangles.end()) {
      group_triangles.emplace(io_tri.mtl, std::vector<lm::io_triangle>{io_tri});
    } else {
      iter->second.emplace_back(io_tri);
    }
  };
  for (auto &tri : triangles) {
    add_triangles(tri);
  }

  for (auto &quad : quads) {
    triangle_type tri1(quad[0], quad[1], quad[2]);
    triangle_type tri2(quad[2], quad[3], quad[0]);

    add_triangles(tri1);
    add_triangles(tri2);
  }
  triangles_ptr.reset();
  points_ptr.reset();
  quads_ptr.reset();

  using SimplifyMesh = lm::texture_mesh;
  std::unique_ptr<SimplifyMesh> _smesh = std::make_unique<SimplifyMesh>();
  SimplifyMesh &smesh = *_smesh;

  int face_count;
  int face_index = 0;
  for (auto kv : group_triangles) {

    int mtl_index = kv.first;
    auto &faces = kv.second;
    face_count += faces.size();

    smesh.textures.emplace_back(std::to_string(mtl_index));

    auto vertex_iter =
        vcg::tri::Allocator<SimplifyMesh>::AddVertices(smesh, faces.size() * 3);
    auto face_iter =
        vcg::tri::Allocator<SimplifyMesh>::AddFaces(smesh, faces.size());

    auto value_face = faces.begin();
    while (face_iter != smesh.face.end()) {
      face_iter->WT(0).U() = mtl_index;
      face_iter->WT(1).U() = mtl_index;
      face_iter->WT(2).U() = mtl_index;

      auto &tri = *value_face;
      (*vertex_iter).P()[0] = tri.vertices[0].v[0];
      (*vertex_iter).P()[1] = tri.vertices[0].v[1];
      (*vertex_iter).P()[2] = tri.vertices[0].v[2];

      (*vertex_iter).T() = face_iter->WT(0);

      auto frist = &(*vertex_iter);
      ++vertex_iter;

      (*vertex_iter).P()[0] = tri.vertices[1].v[0];
      (*vertex_iter).P()[1] = tri.vertices[1].v[1];
      (*vertex_iter).P()[2] = tri.vertices[1].v[2];
      (*vertex_iter).T() = face_iter->WT(1);
      auto second = &(*vertex_iter);
      ++vertex_iter;

      (*vertex_iter).P()[0] = tri.vertices[2].v[0];
      (*vertex_iter).P()[1] = tri.vertices[2].v[1];
      (*vertex_iter).P()[2] = tri.vertices[2].v[2];
      (*vertex_iter).T() = face_iter->WT(2);

      face_iter->V(0) = frist;
      face_iter->V(1) = second;
      face_iter->V(2) = &(*vertex_iter);
      face_iter->mtl = mtl_index;
      face_iter->node = 0;
      face_iter->tex = value_face->tex;
      face_iter->feature_id = value_face->feature_id;

      ++vertex_iter;
      ++value_face;
      ++face_iter;
    }
  }

  group_triangles_ptr.reset();

  vcg::tri::Clean<SimplifyMesh>::RemoveDuplicateVertex(smesh);
  vcg::tri::Allocator<SimplifyMesh>::CompactVertexVector(smesh);
  vcg::tri::Allocator<SimplifyMesh>::CompactFaceVector(smesh);
  vcg::tri::UpdateNormal<SimplifyMesh>::PerVertex(smesh);

  // lock  border for same material
  int locked_border_count = 0;
  {
    struct lock_condition {
      std::set<uint32_t> mtl;
      std::set<globe_feature_id_t> features;

      inline operator bool() const {
        return mtl.size() > 1 || features.size() > 1;
      }
    };
    {
      std::unordered_map<SimplifyMesh::VertexPointer, lock_condition> v_map;

      for (auto &iter : smesh.face) {
        int i = 0;
        while (i < 3) {
          auto f = v_map.find((iter.V(i)));
          if (f == v_map.end()) {
            v_map.emplace(
                (iter.V(i)),
                lock_condition{std::set<uint32_t>{iter.mtl},
                               std::set<globe_feature_id_t>{iter.feature_id}});
          } else {
            f->second.mtl.emplace(iter.mtl);
            f->second.features.emplace(iter.feature_id);
          }
          i++;
        }
      }

      for (auto i : v_map) {
        if (i.second) {
          if (!i.first->IsB())
            locked_border_count++;
          i.first->SetB();
        }
      }
    }
  };

  { // fix seam
    if (lock && false) {
      for (uint32_t i = 0; i < smesh.vert.size(); i++) {
        auto &v = smesh.vert[i];
        if (lock->is_need_lock(vector3<float>{v.P()[0], v.P()[1], v.P()[2]})) {
          if (!v.IsB())
            locked_border_count++;
          v.SetB();
        }
      }
    }
  }

  smesh.lock_border();
  smesh.simplify((face_count - locked_border_count / 3) * fquality +
                     locked_border_count / 3,
                 SimplifyMesh::QUADRICS);

  { // transfrom
    for (uint32_t i = 0; i < smesh.vert.size(); i++) {
      auto &v = smesh.vert[i];
      vector3<float> vertex{v.P()[0], v.P()[1], v.P()[2]};
      vertex *= mat;
      v.P()[0] = vertex.x;
      v.P()[1] = vertex.y;
      v.P()[2] = vertex.z;
    }
  }

#if false
    vcg::tri::io::ExporterOBJ<SimplifyMesh>::Save(smesh, "./test.obj", 0);
#endif
  rettriangles.start().get();

  std::vector<lm::io_triangle> vtriangles(smesh.FN());

  smesh.get_triangles(vtriangles.data(), 0);
  size_t fn_size = smesh.FN();
  _smesh.reset();
  rettriangles.set_elements_per_block(1 << 20);
  rettriangles.resize(fn_size).get();

  {
    size_t index = 0;
    for (auto &iter : vtriangles) {
      lm::io_triangle &io_tri = rettriangles[index++];
      io_tri = iter;

      if (io_tri.tex != -1)
        io_tri.tex += text_offset;

      if (io_tri.mtl != -1)
        io_tri.mtl += mtl_offset;
    }
  }

  std::sort(vtriangles.begin(), vtriangles.end(), [](auto &a, auto &b) {
    return a.mtl < b.mtl || a.tex < b.tex || a.feature_id < b.feature_id;
  });
}

void make_io_triangles(lm::virtual_array<lm::io_triangle> &vtriangles,
                       voxel_pot &pot, const matrix44<double> &mat,
                       size_t text_offset, size_t mtl_offset, double fquality,
                       double isovalue, double adapter) {
  make_io_triangles(vtriangles, pot, mat, text_offset, mtl_offset, nullptr,
                    nullptr, fquality, isovalue, adapter);
}

} // namespace voxelize
} // namespace flywave
