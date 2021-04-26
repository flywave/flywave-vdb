#pragma once

#include <vcg/space/color4.h>
#include <vcg/space/point3.h>

#include <vcg/complex/complex.h>

#include <vcg/math/quadric.h>

#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/topology.h>

#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric_tex.h>

namespace flywave {

class vcg_vertex;
class vcg_edge;
class vcg_face;

struct vcg_used_types
    : public vcg::UsedTypes<vcg::Use<vcg_vertex>::AsVertexType,
                            vcg::Use<vcg_edge>::AsEdgeType,
                            vcg::Use<vcg_face>::AsFaceType> {};

class vcg_vertex
    : public vcg::Vertex<vcg_used_types, vcg::vertex::VFAdj,
                         vcg::vertex::Coord3f, vcg::vertex::Normal3f,
                         vcg::vertex::TexCoord2f, vcg::vertex::Color4b,
                         vcg::vertex::Mark, vcg::vertex::BitFlags> {
public:
  vcg_vertex() { q.SetZero(); }
  vcg::math::Quadric<double> &Qd() { return q; }

private:
  vcg::math::Quadric<double> q;
};

class vcg_vertex_compare {
public:
  bool operator()(const vcg_vertex &va, const vcg_vertex &vb) const {
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

class vcg_edge : public vcg::Edge<vcg_used_types> {};

class vcg_face
    : public vcg::Face<vcg_used_types, vcg::face::VFAdj, vcg::face::Normal3f,
                       vcg::face::WedgeTexCoord2f, vcg::face::VertexRef,
                       vcg::face::BitFlags> {
public:
  uint32_t mtl;

  bool operator<(const vcg_face &t) const { return mtl < t.mtl; }
};

class vcg_mesh
    : public vcg::tri::TriMesh<std::vector<vcg_vertex>, std::vector<vcg_face>> {
public:
  vcg_mesh() = default;
  vcg_mesh(const vcg_mesh &m);

  void save(const char *filename);

  void lock(std::vector<bool> &locked);

  void unlock_border();
  void lock_border();

  inline bool has_border() const { return _has_border; }

  float simplify(uint16_t target_faces);

  float average_distance();

  vcg::Box3f bounding_box();
  vcg::Sphere3f bounding_sphere();

  void transform(vcg::Matrix44f mat);
  void translate(vcg::Point3f p);
  void scale(float s);
  void scale(vcg::Point3f s);
  void clip(vcg::Box3f b);
  void update_normals();
  void update_bounding_box();
  void smooth(int step);
  void compact();
  void update_topologies();
  void clean();

  void sort_face();
  void sort_vert();

protected:
  float quadric_simplify(uint16_t target_faces);

  float edge_length_error();

  bool _has_border{false};
  bool _has_colors{false};
  bool _has_mtls{false};
  bool _has_normals{false};
  bool _has_texs{false};
};

typedef vcg::tri::BasicVertexPair<vcg_vertex> vcg_vertex_pair;

class vcg_tri_edge_collapse
    : public vcg::tri::TriEdgeCollapseQuadric<
          vcg_mesh, vcg_vertex_pair, vcg_tri_edge_collapse,
          vcg::tri::QInfoStandard<vcg_vertex>> {
public:
  typedef vcg::tri::TriEdgeCollapseQuadric<vcg_mesh, vcg_vertex_pair,
                                           vcg_tri_edge_collapse,
                                           vcg::tri::QInfoStandard<vcg_vertex>>
      TECQ;
  typedef vcg_mesh::VertexType::EdgeType EdgeType;
  inline vcg_tri_edge_collapse(const vcg_vertex_pair &p, int i,
                               vcg::BaseParameterClass *pp)
      : TECQ(p, i, pp) {}
};

class vcg_texture_tri_edge_collapse
    : public vcg::tri::TriEdgeCollapseQuadric<
          vcg_mesh, vcg_vertex_pair, vcg_texture_tri_edge_collapse,
          vcg::tri::QInfoStandard<vcg_vertex>> {
public:
  typedef vcg::tri::TriEdgeCollapseQuadric<vcg_mesh, vcg_vertex_pair,
                                           vcg_texture_tri_edge_collapse,
                                           vcg::tri::QInfoStandard<vcg_vertex>>
      TECQ;
  typedef vcg_mesh::VertexType::EdgeType EdgeType;
  inline vcg_texture_tri_edge_collapse(const vcg_vertex_pair &p, int i,
                                       vcg::BaseParameterClass *pp)
      : TECQ(p, i, pp) {}
};

class vcg_texture_tri_edge_collapse_q_tex
    : public vcg::tri::TriEdgeCollapseQuadricTex<
          vcg_mesh, vcg_vertex_pair, vcg_texture_tri_edge_collapse_q_tex,
          vcg::tri::QuadricTexHelper<vcg_mesh>> {
public:
  typedef vcg::tri::TriEdgeCollapseQuadricTex<
      vcg_mesh, vcg_vertex_pair, vcg_texture_tri_edge_collapse_q_tex,
      vcg::tri::QuadricTexHelper<vcg_mesh>>
      TECQ;
  inline vcg_texture_tri_edge_collapse_q_tex(const vcg_vertex_pair &p, int i,
                                             vcg::BaseParameterClass *pp)
      : TECQ(p, i, pp) {}
};

} // namespace flywave
