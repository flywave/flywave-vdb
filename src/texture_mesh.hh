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

class texture_vertex;
class texture_edge;
class texture_face;

struct texture_used_types
    : public vcg::UsedTypes<vcg::Use<texture_vertex>::AsVertexType,
                            vcg::Use<texture_edge>::AsEdgeType,
                            vcg::Use<texture_face>::AsFaceType> {};

class texture_vertex
    : public vcg::Vertex<texture_used_types, vcg::vertex::VFAdj,
                         vcg::vertex::Coord3f, vcg::vertex::Normal3f,
                         vcg::vertex::TexCoord2f, vcg::vertex::Color4b,
                         vcg::vertex::Mark, vcg::vertex::BitFlags> {
public:
  uint32_t node;
  texture_vertex() : node(0) { q.SetZero(); }

  vcg::math::Quadric<double> &Qd() { return q; }

private:
  vcg::math::Quadric<double> q;
};

class texture_vertex_node_compare {
public:
  bool operator()(const texture_vertex &va, const texture_vertex &vb) const {
    return va.node < vb.node;
  }
};

class texture_vertex_compare {
public:
  bool operator()(const texture_vertex &va, const texture_vertex &vb) const {
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

class texture_edge : public vcg::Edge<texture_used_types> {};

class texture_face
    : public vcg::Face<texture_used_types, vcg::face::VFAdj,
                       vcg::face::VertexRef, vcg::face::Normal3f,
                       vcg::face::WedgeTexCoord2f, vcg::face::BitFlags> {
public:
  uint32_t node;
  uint32_t tex;
  uint32_t mtl;
  uint64_t feature_id;
  bool operator<(const texture_face &t) const {
    if (node == t.node)
      return mtl < t.mtl;
    return node < t.node;
  }
};

class texture_mesh : public vcg::tri::TriMesh<std::vector<texture_vertex>,
                                              std::vector<texture_face>> {
protected:
  bool _is_tex = false;
  bool _has_border{false};

  void reset_fv();

public:
  std::string texture;
  texture_mesh() = default;
  texture_mesh(const texture_mesh &m);

  void load(std::vector<std::shared_ptr<flywave::mesh_data>> &mdts);
  void load(voxel_io_triangle *tris, int count);
  void save(const char *filename, uint32_t node);
  
  void lock(std::vector<bool> &locked);
  void lock(bool *locked, int count);

  void get_triangles(struct voxel_io_triangle *triangles, uint32_t node);

  void unlock_border();
  void lock_border();

  inline bool has_border() const { return _has_border; }

  void quadric_simplify_with_tex(uint32_t target);

  void quadric_simplify(uint32_t target);
};

typedef vcg::tri::BasicVertexPair<texture_vertex> texture_vertex_pair;

class texture_tri_edge_collapse
    : public vcg::tri::TriEdgeCollapseQuadric<
          texture_mesh, texture_vertex_pair, texture_tri_edge_collapse,
          vcg::tri::QInfoStandard<texture_vertex>> {
public:
  typedef vcg::tri::TriEdgeCollapseQuadric<
      texture_mesh, texture_vertex_pair, texture_tri_edge_collapse,
      vcg::tri::QInfoStandard<texture_vertex>>
      TECQ;
  typedef texture_mesh::VertexType::EdgeType EdgeType;
  inline texture_tri_edge_collapse(const texture_vertex_pair &p, int i,
                                   vcg::BaseParameterClass *pp)
      : TECQ(p, i, pp) {}
};

class texture_tri_edge_collapse_q_tex
    : public vcg::tri::TriEdgeCollapseQuadricTex<
          texture_mesh, texture_vertex_pair, texture_tri_edge_collapse_q_tex,
          vcg::tri::QuadricTexHelper<texture_mesh>> {
public:
  typedef vcg::tri::TriEdgeCollapseQuadricTex<
      texture_mesh, texture_vertex_pair, texture_tri_edge_collapse_q_tex,
      vcg::tri::QuadricTexHelper<texture_mesh>>
      TECQ;
  inline texture_tri_edge_collapse_q_tex(const texture_vertex_pair &p, int i,
                                         vcg::BaseParameterClass *pp)
      : TECQ(p, i, pp) {}
};

} // namespace flywave
