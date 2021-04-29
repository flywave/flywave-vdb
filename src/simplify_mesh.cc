#include "simplify_mesh.hh"
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

void simplify_mesh::quadric_simplify_with_tex(uint32_t target) {
  _is_tex = true;
  vcg::tri::UpdateNormal<simplify_mesh>::PerFaceNormalized(*this);
  for (auto &f : face) {
    if (f.N().Norm() < 0.01)
      f.N() = vcg::Point3f(1, 0, 0);
  }
  for (uint32_t i = 0; i < face.size(); i++)
    if (!face[i].IsW())
      for (int k = 0; k < 3; k++)
        face[i].V(k)->ClearW();

  vcg::tri::UpdateTopology<simplify_mesh>::VertexFace(*this);

  vcg::tri::TriEdgeCollapseQuadricTexParameter qparams;
  qparams.SetDefaultParams();
  qparams.NormalCheck = true;

  vcg::math::Quadric<double> QZero;
  QZero.SetZero();
  vcg::tri::QuadricTexHelper<simplify_mesh>::QuadricTemp TD3(this->vert, QZero);
  vcg::tri::QuadricTexHelper<simplify_mesh>::TDp3() = &TD3;

  std::vector<std::pair<vcg::TexCoord2<float>, vcg::Quadric5<double>>> qv;

  vcg::tri::QuadricTexHelper<simplify_mesh>::Quadric5Temp TD(this->vert, qv);
  vcg::tri::QuadricTexHelper<simplify_mesh>::TDp() = &TD;

  vcg::LocalOptimization<simplify_mesh> DeciSession(*this, &qparams);
  DeciSession.Init<simplify_tri_edge_collapse_q_tex>();
  DeciSession.SetTargetSimplices(target);
  while (fn > target) {
    DeciSession.DoOptimization();
  }
  DeciSession.Finalize<simplify_tri_edge_collapse_q_tex>();
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

void simplify_mesh::quadric_simplify(uint32_t target) {
  vcg::tri::UpdateTopology<simplify_mesh>::VertexFace(*this);
  vcg::tri::TriEdgeCollapseQuadricParameter qparams;
  qparams.NormalCheck = true;
  vcg::LocalOptimization<simplify_mesh> DeciSession(*this, &qparams);

  DeciSession.Init<simplify_tri_edge_collapse>();
  DeciSession.SetTargetSimplices(target);
  DeciSession.DoOptimization();
  DeciSession.Finalize<simplify_tri_edge_collapse>();
  reset_fv();
}

void simplify_mesh::reset_fv() {
  if (!face.size())
    return;

  vcg::tri::Clean<simplify_mesh>::RemoveDuplicateVertex(*this);
  vcg::tri::Allocator<simplify_mesh>::CompactEveryVector(*this);
  vert.resize(vn);
  face.resize(fn);
}

void simplify_mesh::lock(std::vector<bool> &locked) {
  for (uint32_t i = 0; i < face.size(); i++)
    if (locked[i])
      face[i].ClearW();
}

void simplify_mesh::get_triangles(struct _io_triangle_t *triangles,
                                  uint32_t node) {
  int count = 0;
  for (uint32_t i = 0; i < face.size(); i++) {
    simplify_face &t = face[i];
    if (t.IsD())
      continue;

    struct _io_triangle_t &triangle = triangles[count++];
    for (uint32_t k = 0; k < 3; k++) {
      struct _io_vertex_t &vertex = triangle.vertices[k];
      simplify_vertex &v = *t.V(k);
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

void simplify_mesh::unlock_border() {
  simplify_mesh::VertexIterator vi;
  for (vi = this->vert.begin(); vi != this->vert.end(); ++vi) {
    if ((*vi).IsB())
      (*vi).ClearW();
  }
}

void simplify_mesh::lock_border() {
  simplify_mesh::VertexIterator vi;
  for (vi = this->vert.begin(); vi != this->vert.end(); ++vi) {
    if ((*vi).IsB())
      (*vi).SetW();
  }
}

} // namespace flywave
