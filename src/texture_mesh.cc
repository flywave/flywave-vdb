#include "texture_mesh.hh"
#include "voxelizer_api_impl.hh"

namespace flywave {

class union_find {
public:
  std::vector<int> parents;

  void init(int size) {
    parents.resize(size);
    for (int i = 0; i < size; i++)
      parents[i] = i;
  }

  int root(int p) {
    while (p != parents[p])
      p = parents[p] = parents[parents[p]];
    return p;
  }

  void link(int p0, int p1) {
    int r0 = root(p0);
    int r1 = root(p1);
    parents[r1] = r0;
  }

  int compact(std::vector<int> &node_component) {
    node_component.resize(parents.size());
    std::map<int, int> remap;
    for (size_t i = 0; i < parents.size(); i++) {
      int root = i;
      while (root != parents[root])
        root = parents[root];
      parents[i] = root;
      node_component[i] = remap.emplace(root, remap.size()).first->second;
    }
    return remap.size();
  }
};

texture_mesh::texture_mesh(const texture_mesh &m) {
  vcg::tri::Append<texture_mesh, texture_mesh>::MeshCopy(
      *this, const_cast<texture_mesh &>(m));
}

void texture_mesh::load(voxel_io_triangle *tris, int count) {
  vcg::tri::Allocator<texture_mesh>::AddVertices(*this, count * 3);
  vcg::tri::Allocator<texture_mesh>::AddFaces(*this, count);

  for (uint32_t i = 0; i < count; i++) {
    voxel_io_triangle &triangle = tris[i];
    texture_face &f = face[i];
    for (uint32_t k = 0; k < 3; k++) {
      voxel_io_vertex &in = triangle.vertices[k];
      texture_vertex &v = vert[i * 3 + k];
      if (in.b) {
        v.SetB();
        _has_border = true;
      }
      v.P() = vcg::Point3f(in.v[0], in.v[1], in.v[2]);
      v.C() = vcg::Color4b(in.c[0], in.c[1], in.c[2], in.c[3]);
      f.V(k) = &v;
      f.WT(k).U() = in.t[0];
      f.WT(k).V() = in.t[1];
      assert(!std::isnan(in.t[0]));
      assert(!std::isnan(in.t[1]));
    }
    f.node = triangle.node;
    f.tex = triangle.tex;
    f.mtl = triangle.mtl;
  }

  vcg::tri::Clean<texture_mesh>::RemoveDuplicateVertex(*this);
  vcg::tri::Clean<texture_mesh>::RemoveDuplicateFace(*this);
  vcg::tri::Clean<texture_mesh>::RemoveZeroAreaFace(*this);
  vcg::tri::Clean<texture_mesh>::RemoveUnreferencedVertex(*this);
  vcg::tri::UpdateNormal<texture_mesh>::PerFaceNormalized(*this);
  vcg::tri::Allocator<texture_mesh>::CompactVertexVector(*this);
  vcg::tri::Allocator<texture_mesh>::CompactFaceVector(*this);
  vcg::tri::UpdateNormal<texture_mesh>::PerVertex(*this);
}

void texture_mesh::quadric_simplify_with_tex(uint32_t target) {
  _is_tex = true;
  vcg::tri::UpdateNormal<texture_mesh>::PerFaceNormalized(*this);
  for (auto &f : face) {
    if (f.N().Norm() < 0.01)
      f.N() = vcg::Point3f(1, 0, 0);
  }
  for (uint32_t i = 0; i < face.size(); i++)
    if (!face[i].IsW())
      for (int k = 0; k < 3; k++)
        face[i].V(k)->ClearW();

  vcg::tri::UpdateTopology<texture_mesh>::VertexFace(*this);

  vcg::tri::TriEdgeCollapseQuadricTexParameter qparams;
  qparams.SetDefaultParams();
  qparams.NormalCheck = true;

  vcg::math::Quadric<double> QZero;
  QZero.SetZero();
  vcg::tri::QuadricTexHelper<texture_mesh>::QuadricTemp TD3(this->vert, QZero);
  vcg::tri::QuadricTexHelper<texture_mesh>::TDp3() = &TD3;

  std::vector<std::pair<vcg::TexCoord2<float>, vcg::Quadric5<double>>> qv;

  vcg::tri::QuadricTexHelper<texture_mesh>::Quadric5Temp TD(this->vert, qv);
  vcg::tri::QuadricTexHelper<texture_mesh>::TDp() = &TD;

  vcg::LocalOptimization<texture_mesh> DeciSession(*this, &qparams);
  DeciSession.Init<texture_tri_edge_collapse_q_tex>();
  DeciSession.SetTargetSimplices(target);
  while (fn > target) {
    DeciSession.DoOptimization();
  }
  DeciSession.Finalize<texture_tri_edge_collapse_q_tex>();
  reset_fv();

  std::vector<bool> visited(vn, false);
  for (int i = 0; i < fn; i++)
    if (!(face[i]).IsD()) {
      auto &f = face[i];
      for (int k = 0; k < 3; k++) {
        auto &nv = *f.V(k);
        int index = &nv - &*vert.begin();
        if (!visited[index]) {
          nv.T() = f.WT(k);
        }
      }
    }
}

void texture_mesh::quadric_simplify(uint32_t target) {
  vcg::tri::UpdateTopology<texture_mesh>::VertexFace(*this);
  vcg::tri::TriEdgeCollapseQuadricParameter qparams;
  qparams.NormalCheck = true;
  vcg::LocalOptimization<texture_mesh> DeciSession(*this, &qparams);

  DeciSession.Init<texture_tri_edge_collapse>();
  DeciSession.SetTargetSimplices(target);
  DeciSession.DoOptimization();
  DeciSession.Finalize<texture_tri_edge_collapse>();
  reset_fv();
}

void texture_mesh::reset_fv() {
  if (!face.size())
    return;

  vcg::tri::Clean<texture_mesh>::RemoveDuplicateVertex(*this);
  vcg::tri::Allocator<texture_mesh>::CompactEveryVector(*this);
  vert.resize(vn);
  face.resize(fn);
}

void texture_mesh::lock(std::vector<bool> &locked) {
  for (uint32_t i = 0; i < face.size(); i++)
    if (locked[i])
      face[i].ClearW();
}

void texture_mesh::lock(bool *locked, int count) {
  assert(count == face.size());
  for (uint32_t i = 0; i < count; i++)
    if (locked[i])
      face[i].ClearW();
}

void texture_mesh::get_triangles(struct voxel_io_triangle *triangles,
                                 uint32_t node) {
  int count = 0;
  for (uint32_t i = 0; i < face.size(); i++) {
    texture_face &t = face[i];
    if (t.IsD())
      continue;

    struct voxel_io_triangle &triangle = triangles[count++];
    for (uint32_t k = 0; k < 3; k++) {
      struct voxel_io_vertex &vertex = triangle.vertices[k];
      texture_vertex &v = *t.V(k);
      vertex.v[0] = v.P()[0];
      vertex.v[1] = v.P()[1];
      vertex.v[2] = v.P()[2];
      vertex.c[0] = v.C()[0];
      vertex.c[1] = v.C()[1];
      vertex.c[2] = v.C()[2];
      vertex.c[3] = v.C()[3];
      vertex.t[0] = t.WT(k).U();
      vertex.t[1] = t.WT(k).V();
      vertex.b = v.IsB();
    }
    triangle.node = node;
    triangle.tex = t.tex;
    triangle.mtl = t.mtl;
    triangle.feature_id = t.feature_id;
  }
}

void texture_mesh::unlock_border() {
  texture_mesh::VertexIterator vi;
  for (vi = this->vert.begin(); vi != this->vert.end(); ++vi) {
    if ((*vi).IsB())
      (*vi).ClearW();
  }
}

void texture_mesh::lock_border() {
  texture_mesh::VertexIterator vi;
  for (vi = this->vert.begin(); vi != this->vert.end(); ++vi) {
    if ((*vi).IsB())
      (*vi).SetW();
  }
}

void texture_mesh::load(std::vector<std::shared_ptr<flywave::mesh_data>> &mdts) {
  size_t s = 0;
  size_t vers = 0;
  for (auto &mdt : mdts) {
    auto &verts = mdt->vertices();
    vers += verts.size();

    auto &fs = mdt->mtl_faces_map();
    auto bg = fs.begin();
    while (bg != fs.end()) {
      s += bg->second.size();
      bg++;
    }
  }

  auto fi = vcg::tri::Allocator<texture_mesh>::AddFaces(*this, s);
  vcg::tri::Allocator<texture_mesh>::AddVertices(*this, vers);

  int befor_size = 0;
  for (auto &mdt : mdts) {
    auto &verts = mdt->vertices();
    auto &texs = mdt->texcoords();
    auto &fs = mdt->mtl_faces_map();
    auto bg = fs.begin();
    std::vector<bool> visit_map(verts.size(), false);

    bg = fs.begin();
    while (bg != fs.end()) {
      auto &faces = bg->second;

      for (int n = 0; n < faces.size(); n++) {
        auto &f_index = faces[n];
        for (int i = 0; i < 3; i++) {
          auto ver_index = f_index[i];
          auto &ver = verts[ver_index];
          auto &vi = this->vert[befor_size + f_index[i]];
          vi.P() = texture_mesh::CoordType(
              static_cast<texture_mesh::ScalarType>(ver[0]),
              static_cast<texture_mesh::ScalarType>(ver[1]),
              static_cast<texture_mesh::ScalarType>(ver[2]));
          if (mdt->has_texcoord()) {
            float u = texs[f_index[i]][0];
            float v = texs[f_index[i]][1];
            vi.T() = vcg::TexCoord2f{u, v};
            (*fi).WT(i).U() = u;
            (*fi).WT(i).V() = v;
          }
          (*fi).V(i) = &vi;
        }
        (*fi).mtl = bg->first;
        fi++;
      }
      bg++;
    }
    befor_size += verts.size();
  }
  vcg::tri::Clean<texture_mesh>::RemoveDuplicateVertex(*this);
  vcg::tri::Allocator<texture_mesh>::CompactVertexVector(*this);
  vcg::tri::Allocator<texture_mesh>::CompactFaceVector(*this);
}

} // namespace flywave
