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

  static texture2d<color4<uint8_t>>::ptr
  project_to_image(std::vector<sampler_type> points,
                   const triangle_projection &prj, texture2d<color4i>::ptr);

  size_t pixel_size() const { return _query->pixel_count(); }

private:
  std::unique_ptr<triangle_range_query<pixel_grid>> _query;
};

textute_foundry::textute_foundry(vertex_grid::Ptr cgrid, pixel_grid::Ptr pgrid,
                                 float tquality, float pixel_pad)
    : _query(std::make_unique<impl>(cgrid, pgrid)), _grid(cgrid),
      _texture_quality(tquality), _pixel_pad(pixel_pad) {}

texture2d<color4<uint8_t>>::ptr
textute_foundry::extract(const fmesh_tri_patch &tri,
                         texture2d<color4i>::ptr texture) const {
  fmesh_tri_patch ptri = tri;
  ptri.p1 = _grid->transform().world_to_index(tri.p1);
  ptri.p2 = _grid->transform().world_to_index(tri.p2);
  ptri.p3 = _grid->transform().world_to_index(tri.p3);

  triangle_projection proj(ptri, _pixel_pad);
  return impl::project_to_image(_query->extract(proj.to_voxels()), proj,
                                texture);
}

size_t textute_foundry::pixel_size() const { return _query->pixel_size(); }

textute_foundry::~textute_foundry() = default;

texture2d<color4<uint8_t>>::ptr
textute_foundry::impl::project_to_image(std::vector<sampler_type> points,
                                        const triangle_projection &prj,
                                        texture2d<color4i>::ptr img) {

  auto h = img->height();

  for (auto &it : points) {
    auto uv = prj.point_to_uv(it._tri_coord);
    if (uv.x >= 0 && uv.y >= 0) {

      uint64_t min_x = std::min(uint64_t(std::floor(uv.x)), img->width() - 1);
      uint64_t min_y = std::min(uint64_t(std::floor(h - uv.y - 1)), h - 1);
      uint64_t max_x = std::min(uint64_t(std::ceil(uv.x)), img->width() - 1);
      uint64_t max_y = std::min(uint64_t(std::ceil(h - uv.y - 1)), h - 1);

      // auto zero = color4i();
      // if (img->color(min_x, min_y) == zero) {
      img->color(min_x, min_y) = it._value._data._color;
      // }
      // if (img->color(max_x, max_y) == zero) {
      img->color(max_x, max_y) = it._value._data._color;
      // }
      // if (img->color(min_x, max_y) == zero) {
      img->color(min_x, max_y) = it._value._data._color;
      // }
      // if (img->color(max_x, min_y) == zero) {
      img->color(max_x, min_y) = it._value._data._color;
      // }
      // if (x == 325 && y == 1077) {
      //   FLYWAVE_ASSERT(true);
      // }
      // img->color(x, y) = it._value._data._color;
    }
  }
  return img;
}

namespace {
class in_setting_volume_mesh_func
    : public vdb::tools::setting_volume_mesh_func {
public:
  in_setting_volume_mesh_func(shared_ptr<seam_repair> srepair)
      : _seam_repair(srepair) {}

  void setting(vdb::tools::volume_to_mesh_t &mt) override {
    mt.set_in_box_border_sampling(
        flywave::make_shared<vdb::tools::box_border_sampling>(
            _seam_repair->_seam_vertexs, _seam_repair->_seam_index_tree));
  }

private:
  shared_ptr<seam_repair> _seam_repair;
};

class out_setting_volume_mesh_func
    : public in_setting_volume_mesh_func::setting_volume_mesh_func {
public:
  out_setting_volume_mesh_func(const seam_box_setting &setting)
      : _seam_box_setting(setting) {}

  void setting(vdb::tools::volume_to_mesh_t &mt) override {
    mt.set_box_border_mark(_seam_box_setting._box,
                           *_seam_box_setting._transform, _seam_box_setting._up,
                           _seam_box_setting._left);

    _seam_repair->_seam_index_tree =
        mt.get_out_box_border_sampling()->get_tree();

    _seam_repair->_seam_vertexs =
        mt.get_out_box_border_sampling()->get_seam_points();
  }

  shared_ptr<seam_repair> get_seam_repair() { return _seam_repair; }

private:
  shared_ptr<seam_repair> _seam_repair;
  seam_box_setting _seam_box_setting;
};
} // namespace

future<> triangle_foundry::make_mesh(std::vector<vertext_type> &points,
                                     std::vector<triangle_type> &tri,
                                     std::vector<quad_type> &quads,
                                     double isovalue, double adapter) {
  if (_seam_repair) {
    return vdb::tools::volume_to_mesh(
        *_grid, points, quads, isovalue,
        make_shared<in_setting_volume_mesh_func>(_seam_repair));
  }
  return vdb::tools::volume_to_mesh(*_grid, points, tri, quads, isovalue,
                                    nullptr, adapter);
}

future<shared_ptr<seam_repair>> triangle_foundry::make_mesh_mark_seam(
    const seam_box_setting &mbox, std::vector<vertext_type> &points,
    std::vector<triangle_type> &tri, std::vector<quad_type> &quads,
    double isovalue, double adapter) {
  auto ptr = make_shared<out_setting_volume_mesh_func>(mbox);
  return vdb::tools::volume_to_mesh(*_grid, points, tri, quads, isovalue, ptr,
                                    adapter)
      .then([ptr] { return ptr->get_seam_repair(); });
}

future<> make_io_triangles(lm::virtual_array<lm::io_triangle> &rettriangles,
                           voxel_pot &pot, shared_ptr<seam_repair> repair,
                           double fquality, double isovalue, double adapter) {
  return async([&pot, isovalue, adapter, repair, fquality, &rettriangles] {
    shared_ptr<triangle_foundry> vfoundry;
    if (repair) {
      vfoundry = make_shared<triangle_foundry>(pot.voxel_grid(), repair);
    } else {
      vfoundry = make_shared<triangle_foundry>(pot.voxel_grid());
    }
    std::vector<vertext_type> points;
    std::vector<triangle_type> triangles;
    std::vector<quad_type> quads;

    vfoundry->make_mesh(points, triangles, quads, isovalue, adapter).get();

    // std::vector<lm::io_triangle> vtriangles(triangles.size() +
    //                                         quads.size() * 2);

    std::unordered_map<int, std::vector<lm::io_triangle>> group_triangles;
    size_t count = 0;
    auto assess = pot.pixel_grid()->get_accessor();
    auto add_triangles = [&count, &points, &pot, &group_triangles, &assess,
                          &repair](triangle_type &tri) {

      auto &v0 = points[tri[0]];
      auto &v1 = points[tri[1]];
      auto &v2 = points[tri[2]];

      struct mtsetting {
        int tex;
        int mtl;
        globe_feature_id_t _globe_feature_id;
      };

      auto get_mt_config = [&](const vertext_type &p) {
        auto pt = pot.voxel_resolution()->world_to_index(p);
        auto &pix = assess.get_value(openvdb::math::Coord(pt[0], pt[1], pt[2]));

        mtsetting mtconfig;

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
        FLYWAVE_ASSERT(pot.features().to_globe_feature_id(
            pix._data._feature_id, mtconfig._globe_feature_id));
        return mtconfig;
      };

      lm::io_triangle io_tri;

      io_tri.node = 0;
      {
        mtsetting vp0 = get_mt_config(v0);
        mtsetting vp1 = get_mt_config(v1);
        mtsetting vp2 = get_mt_config(v2);

        if (vp0.tex == 0) {
          io_tri.tex = vp0.tex;
          io_tri.mtl = vp0.mtl;
          io_tri.feature_id = vp0._globe_feature_id;
        } else if (vp1.tex == 0) {
          io_tri.tex = vp1.tex;
          io_tri.mtl = vp1.mtl;
          io_tri.feature_id = vp1._globe_feature_id;
        } else if (vp2.tex == 0) {
          io_tri.tex = vp2.tex;
          io_tri.mtl = vp2.mtl;
          io_tri.feature_id = vp2._globe_feature_id;
        } else {
          io_tri.tex = vp0.tex;
          io_tri.mtl = vp0.mtl;
          io_tri.feature_id = vp0._globe_feature_id;
        }
      }

      auto pt = pot.voxel_resolution()->world_to_index(v0);
      auto pt1 = pot.voxel_resolution()->world_to_index(v1);
      auto pt2 = pot.voxel_resolution()->world_to_index(v2);

      lm::io_vertex _v0{};
      lm::io_vertex _v1{};
      lm::io_vertex _v2{};

      if (repair) {
        _v0.b = repair->_seam_index_tree->is_value_on(
            openvdb::math::Coord(pt[0], pt[1], pt[2]));
        _v1.b = repair->_seam_index_tree->is_value_on(
            openvdb::math::Coord(pt1[0], pt1[1], pt1[2]));
        _v2.b = repair->_seam_index_tree->is_value_on(
            openvdb::math::Coord(pt2[0], pt2[1], pt2[2]));
      }
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
        group_triangles.emplace(io_tri.mtl,
                                std::vector<lm::io_triangle>{io_tri});
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
    using SimplifyMesh = lm::texture_mesh;
    SimplifyMesh smesh;

    int face_count;
    int face_index = 0;
    for (auto kv : group_triangles) {

      int mtl_index = kv.first;
      auto &faces = kv.second;
      face_count += faces.size();

      smesh.textures.emplace_back(std::to_string(mtl_index));

      auto vertex_iter = vcg::tri::Allocator<SimplifyMesh>::AddVertices(
          smesh, faces.size() * 3);
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

    // uint32_t size = vtriangles.size();
    // lm::io_soup soup(vtriangles.data(), &size, vtriangles.size());

    // smesh.load(soup);
    vcg::tri::Clean<SimplifyMesh>::RemoveDuplicateVertex(smesh);
    vcg::tri::Allocator<SimplifyMesh>::CompactVertexVector(smesh);
    vcg::tri::Allocator<SimplifyMesh>::CompactFaceVector(smesh);
    vcg::tri::UpdateNormal<SimplifyMesh>::PerVertex(smesh);

    struct lock_condition {
      std::set<uint32_t> mtl;
      std::set<globe_feature_id_t> features;

      inline operator bool() const {
        return mtl.size() > 1 || features.size() > 1;
      }
    };

    std::map<SimplifyMesh::VertexPointer, lock_condition> v_map;

    for (auto iter : smesh.face) {
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
      if (i.second)
        i.first->SetB();
    }

    smesh.lock_border();
    smesh.simplify(face_count * fquality, SimplifyMesh::QUADRICS);

    // smesh.unlock_border();

#if false
    vcg::tri::io::ExporterOBJ<SimplifyMesh>::Save(smesh, "./test.obj", 0);
#endif
    rettriangles.start().get();

    std::vector<lm::io_triangle> vtriangles(smesh.FN());

    smesh.get_triangles(vtriangles.data(), 0);

    std::sort(vtriangles.begin(), vtriangles.end(), [](auto &a, auto &b) {
      return a.feature_id < b.feature_id || a.mtl < b.mtl;
    });

    rettriangles.set_elements_per_block(1 << 20);
    rettriangles.resize(smesh.FN()).get();
    {
      size_t index = 0;
      for (auto iter : vtriangles) {
        lm::io_triangle &io_tri = rettriangles[index++];
        io_tri = iter;
      }
    }
  });
}

future<> make_io_triangles(lm::virtual_array<lm::io_triangle> &vtriangles,
                           voxel_pot &pot, double fquality, double isovalue,
                           double adapter) {
  return make_io_triangles(vtriangles, pot, nullptr, fquality, isovalue,
                           adapter);
}

} // namespace voxelize
} // namespace flywave
