#pragma once

#include <vcg/space/box3.h>
#include <vcg/space/color4.h>
#include <vcg/space/point3.h>

#include <vcg/complex/complex.h>

#include <vcg/math/quadric.h>

#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/clip.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/topology.h>

#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric_tex.h>

#include "texture2d.hh"
#include "voxelizer_api_impl.hh"

namespace flywave {

class simplify_vertex;
class simplify_edge;
class simplify_face;

struct simplify_used_types
    : public vcg::UsedTypes<vcg::Use<simplify_vertex>::AsVertexType,
                            vcg::Use<simplify_edge>::AsEdgeType,
                            vcg::Use<simplify_face>::AsFaceType> {};

class simplify_vertex
    : public vcg::Vertex<simplify_used_types, vcg::vertex::VFAdj,
                         vcg::vertex::Coord3f, vcg::vertex::Normal3f,
                         vcg::vertex::TexCoord2f, vcg::vertex::Color4b,
                         vcg::vertex::Mark, vcg::vertex::BitFlags> {
public:
  uint32_t node;
  simplify_vertex() : node(0) { q.SetZero(); }
  vcg::math::Quadric<double> &Qd() { return q; }

private:
  vcg::math::Quadric<double> q;
};

class simplify_vertex_node_compare {
public:
  bool operator()(const simplify_vertex &va, const simplify_vertex &vb) const {
    return va.node < vb.node;
  }
};

class simplify_vertex_compare {
public:
  bool operator()(const simplify_vertex &va, const simplify_vertex &vb) const {
    const vcg::Point3f &a = va.cP();
    const vcg::Point3f &b = vb.cP();
    if (a[2] < b[2])
      return true;
    if (a[2] > b[2])
      return false;
    if (a[1] < b[1])
      return true;
    if (a[1] > b[1])
      return false;
    return a[0] < b[0];
  }
};

class simplify_edge : public vcg::Edge<simplify_used_types> {};

class simplify_face
    : public vcg::Face<simplify_used_types, vcg::face::VFAdj,
                       vcg::face::VertexRef, vcg::face::Normal3f,
                       vcg::face::WedgeTexCoord2f, vcg::face::BitFlags> {
public:
  uint32_t node;
  uint32_t tex;
  uint32_t mtl;
  uint64_t feature_id;
  bool operator<(const simplify_face &t) const {
    if (node == t.node)
      return mtl < t.mtl;
    return node < t.node;
  }
};

class simplify_mesh : public vcg::tri::TriMesh<std::vector<simplify_vertex>,
                                               std::vector<simplify_face>> {
protected:
  bool _is_tex = false;

  void reset_fv();

public:
  std::string texture;
  simplify_mesh() = default;
  simplify_mesh(const simplify_mesh &m);

  void lock(std::vector<bool> &locked);
  void get_triangles(struct _triangle_t *triangles, uint32_t node);

  void unlock_border();
  void lock_border();

  void quadric_simplify_with_tex(uint32_t target);

  void quadric_simplify(uint32_t target);
};

typedef vcg::tri::BasicVertexPair<simplify_vertex> simplify_vertex_pair;

class simplify_tri_edge_collapse
    : public vcg::tri::TriEdgeCollapseQuadric<
          simplify_mesh, simplify_vertex_pair, simplify_tri_edge_collapse,
          vcg::tri::QInfoStandard<simplify_vertex>> {
public:
  typedef vcg::tri::TriEdgeCollapseQuadric<
      simplify_mesh, simplify_vertex_pair, simplify_tri_edge_collapse,
      vcg::tri::QInfoStandard<simplify_vertex>>
      TECQ;
  typedef simplify_mesh::VertexType::EdgeType EdgeType;
  inline simplify_tri_edge_collapse(const simplify_vertex_pair &p, int i,
                                    vcg::BaseParameterClass *pp)
      : TECQ(p, i, pp) {}
};

class simplify_tri_edge_collapse_q_tex
    : public vcg::tri::TriEdgeCollapseQuadricTex<
          simplify_mesh, simplify_vertex_pair, simplify_tri_edge_collapse_q_tex,
          vcg::tri::QuadricTexHelper<simplify_mesh>> {
public:
  typedef vcg::tri::TriEdgeCollapseQuadricTex<
      simplify_mesh, simplify_vertex_pair, simplify_tri_edge_collapse_q_tex,
      vcg::tri::QuadricTexHelper<simplify_mesh>>
      TECQ;
  inline simplify_tri_edge_collapse_q_tex(const simplify_vertex_pair &p, int i,
                                          vcg::BaseParameterClass *pp)
      : TECQ(p, i, pp) {}
};

} // namespace flywave
