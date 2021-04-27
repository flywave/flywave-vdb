#include "vcg_mesh.hh"

#include <vcg/space/index/kdtree/kdtree.h>

#include <wrap/io_trimesh/export_obj.h>

#include <vcg/complex/algorithms/clip.h>
#include <vcg/complex/algorithms/create/ball_pivoting.h>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/refine.h>
#include <vcg/complex/algorithms/refine_loop.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/update/position.h>

namespace flywave {

vcg_mesh::vcg_mesh(const vcg_mesh &m) {
  vcg::tri::Append<vcg_mesh, vcg_mesh>::MeshCopy(*this,
                                                 const_cast<vcg_mesh &>(m));
}

vcg_mesh::PerMeshAttributeHandle<std::vector<vcg::tri::io::Material>>
get_material_vector_attribute(vcg_mesh &m) {
  return vcg::tri::Allocator<vcg_mesh>::GetPerMeshAttribute<
      std::vector<vcg::tri::io::Material>>(m, "materialVector");
}

bool has_material_vector_attribute(vcg_mesh &m) {
  return vcg::tri::Allocator<vcg_mesh>::IsValidHandle<
      std::vector<vcg::tri::io::Material>>(
      m, vcg::tri::Allocator<vcg_mesh>::FindPerMeshAttribute<
             std::vector<vcg::tri::io::Material>>(m, "materialVector"));
}

vcg_mesh::PerFaceAttributeHandle<int>
get_material_index_attribute(vcg_mesh &m) {
  return vcg::tri::Allocator<vcg_mesh>::GetPerFaceAttribute<int>(
      m, "materialIndex");
}

bool has_material_index_attribute(vcg_mesh &m) {
  return vcg::tri::Allocator<vcg_mesh>::IsValidHandle<int>(
      m, vcg::tri::Allocator<vcg_mesh>::FindPerFaceAttribute<int>(
             m, "materialIndex"));
}

void vcg_mesh::save(const char *filename) {
  if (_has_texs) {
    typedef vcg_mesh::PerMeshAttributeHandle<
        std::vector<vcg::tri::io::Material>>
        material_type;
    typedef vcg_mesh::PerFaceAttributeHandle<int> material_index_type;

    if (!has_material_vector_attribute(*this)) {
      material_index_type mindexs = get_material_index_attribute(*this);

      for (uint32_t i = 0; i < face.size(); i++) {
        mindexs[i] = face[i].mtl;
      }

      std::vector<vcg::tri::io::Material> &mats =
          get_material_vector_attribute(*this)();
      for (int texindex = 0; texindex < textures.size(); texindex++) {
        vcg::tri::io::Material mat;
        mat.index = texindex;
        mat.map_Kd = textures[texindex];
        mats.emplace_back(mat);
      }
    }
    vcg::tri::io::ExporterOBJ<vcg_mesh>::Save(
        *this, filename, vcg::tri::io::Mask::IOM_VERTTEXCOORD);
  } else {
    vcg::tri::io::ExporterOBJ<vcg_mesh>::Save(*this, filename, 0);
  }
}

float vcg_mesh::simplify(uint16_t target_faces) {
  for (uint32_t i = 0; i < face.size(); i++)
    if (!face[i].IsW())
      for (int k = 0; k < 3; k++)
        face[i].V(k)->ClearW();

  float error = quadric_simplify(target_faces);

  for (uint32_t i = 0; i < vert.size(); i++)
    vert[i].SetW();
  for (uint32_t i = 0; i < face.size(); i++)
    face[i].SetW();

  return error;
}

float vcg_mesh::average_distance() {
  vcg::Box3f box;
  for (int i = 0; i < VN(); i++)
    box.Add(vert[i].cP());

  float area = pow(box.Volume(), 2.0 / 3.0);
  return 8 * pow(area / VN(), 0.5);

  if (VN() <= 5)
    return 0;

  vcg::VertexConstDataWrapper<vcg_mesh> ww(*this);

  vcg::KdTree<float> tree(ww);

  vcg::KdTree<float>::PriorityQueue result;

  float avgDist = 0;
  int count = 0;
  for (int j = 0; j < VN(); j++) {
    tree.doQueryK(vert[j].cP(), 5, result);
    int neighbours = result.getNofElements();
    float m = 0;
    for (int i = 0; i < neighbours; i++) {
      int neightId = result.getIndex(i);
      float d = Distance(vert[j].cP(), vert[neightId].cP());
      if (d > m)
        m = d;
    }
    avgDist += m;
    count++;
  }
  return avgDist / count;
}

vcg::Sphere3f vcg_mesh::bounding_sphere() {
  std::vector<vcg::Point3f> vertices(vert.size());
  for (uint32_t i = 0; i < vert.size(); i++)
    vertices[i] = vert[i].P();
  vcg::Sphere3f sphere;
  sphere.CreateTight(vertices);
  return sphere;
}

vcg::Box3f vcg_mesh::bounding_box() {
  vcg::tri::UpdateBounding<vcg_mesh>::Box(*this);
  return this->bbox;
}

float vcg_mesh::quadric_simplify(uint16_t target) {
  if (_has_texs) {
    vcg::tri::UpdateNormal<vcg_mesh>::PerFaceNormalized(*this);
    for (auto &f : face) {
      if (f.N().Norm() < 0.01)
        f.N() = vcg::Point3f(1, 0, 0);
    }
    vcg::tri::UpdateTopology<vcg_mesh>::VertexFace(*this);

    vcg::tri::TriEdgeCollapseQuadricTexParameter qparams;
    qparams.SetDefaultParams();
    qparams.NormalCheck = true;

    vcg::math::Quadric<double> QZero;
    QZero.SetZero();
    vcg::tri::QuadricTexHelper<vcg_mesh>::QuadricTemp TD3(this->vert, QZero);
    vcg::tri::QuadricTexHelper<vcg_mesh>::TDp3() = &TD3;

    std::vector<std::pair<vcg::TexCoord2<float>, vcg::Quadric5<double>>> qv;

    vcg::tri::QuadricTexHelper<vcg_mesh>::Quadric5Temp TD(this->vert, qv);
    vcg::tri::QuadricTexHelper<vcg_mesh>::TDp() = &TD;

    vcg::LocalOptimization<vcg_mesh> deciSession(*this, &qparams);

    deciSession.Init<vcg_texture_tri_edge_collapse_q_tex>();
    deciSession.SetTargetSimplices(target);
    deciSession.DoOptimization();
    deciSession.Finalize<vcg_texture_tri_edge_collapse_q_tex>();
  } else {
    vcg::tri::UpdateTopology<vcg_mesh>::VertexFace(*this);
    vcg::tri::TriEdgeCollapseQuadricParameter qparams;
    qparams.NormalCheck = true;
    vcg::LocalOptimization<vcg_mesh> DeciSession(*this, &qparams);

    DeciSession.Init<vcg_tri_edge_collapse>();
    DeciSession.SetTargetSimplices(target);
    DeciSession.DoOptimization();
  }
  return edge_length_error();
}

float vcg_mesh::edge_length_error() {
  if (!face.size())
    return 0;
  float error = 0;
  int recount = 0;
  for (unsigned int i = 0; i < face.size(); i++) {
    vcg_face &f = face[i];
    if (f.IsD())
      continue;
    for (int k = 0; k < 3; k++) {
      float err = (f.cV(k)->cP() - f.cV((k + 1) % 3)->cP()).SquaredNorm();
      error += err;
      recount++;
    }
  }
  return std::sqrt(error / recount);
}

void vcg_mesh::sort_face() { std::sort(face.begin(), face.end()); }

void vcg_mesh::sort_vert() {
  std::sort(vert.begin(), vert.end(), vcg_vertex_compare());
}

void vcg_mesh::transform(vcg::Matrix44f mat) {
  vcg::tri::UpdatePosition<vcg_mesh>::Matrix(*this, mat);
}

void vcg_mesh::translate(vcg::Point3f p) {
  vcg::tri::UpdatePosition<vcg_mesh>::Translate(*this, p);
}

void vcg_mesh::scale(float s) {
  vcg::tri::UpdatePosition<vcg_mesh>::Scale(*this, s);
}

void vcg_mesh::scale(vcg::Point3f s) {
  vcg::tri::UpdatePosition<vcg_mesh>::Scale(*this, s);
}

void vcg_mesh::clip(vcg::Box3f b) {
  vcg::tri::GenericVertexInterpolator<vcg_mesh> interp(*this);
  vcg::tri::TriMeshClipper<vcg_mesh>::Box(b, interp, *this);
  vcg::tri::UpdateBounding<vcg_mesh>::Box(*this);
}

void vcg_mesh::update_normals() {
  if (this->fn > 0) {
    vcg::tri::UpdateNormal<vcg_mesh>::PerFaceNormalized(*this);
    vcg::tri::UpdateNormal<vcg_mesh>::PerVertexAngleWeighted(*this);
  }
}

void vcg_mesh::update_bounding_box() {
  vcg::tri::UpdateBounding<vcg_mesh>::Box(*this);
}

template <class MeshType> void update_topologies(MeshType *mesh) {
  vcg::tri::UpdateTopology<MeshType>::FaceFace(*mesh);
  vcg::tri::UpdateTopology<MeshType>::VertexFace(*mesh);
  vcg::tri::UpdateFlags<MeshType>::FaceBorderFromFF(*mesh);
  vcg::tri::UpdateFlags<MeshType>::VertexBorderFromFaceBorder(*mesh);
}

void vcg_mesh::update_topologies() {
  flywave::update_topologies<vcg_mesh>(this);
}

void vcg_mesh::clean() {
  vcg::tri::Clean<vcg_mesh>::RemoveDuplicateVertex(*this);
  vcg::tri::Clean<vcg_mesh>::RemoveDuplicateFace(*this);
  vcg::tri::Clean<vcg_mesh>::RemoveZeroAreaFace(*this);
  vcg::tri::Clean<vcg_mesh>::RemoveUnreferencedVertex(*this);
  vcg::tri::UpdateTopology<vcg_mesh>::FaceFace(*this);
  vcg::tri::Clean<vcg_mesh>::RemoveNonManifoldFace(*this);
  vcg::tri::UpdateTopology<vcg_mesh>::FaceFace(*this);
}

void vcg_mesh::compact() {
  clean();

  vcg::tri::Allocator<vcg_mesh>::CompactVertexVector(*this);
  vcg::tri::Allocator<vcg_mesh>::CompactFaceVector(*this);
}

void vcg_mesh::smooth(int step) {
  vcg::tri::RequirePerVertexNormal(*this);
  vcg::tri::UpdateTopology<vcg_mesh>::VertexFace(*this);

  vcg::tri::Smooth<vcg_mesh>::VertexCoordLaplacian(*this, step, false, true);
  vcg::tri::UpdateNormal<vcg_mesh>::PerVertexPerFace(*this);
}

} // namespace flywave
