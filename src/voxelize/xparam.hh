#pragma once
#include <flywave/mesh/mesh_graph.hh>
#include <flywave/mesh/mesh_uv.hh>
#include <flywave/mesh/parameterization.hh>
#include <flywave/mesh/texture_optimization.hh>

#include <xatlas/src/xatlas.h>

#include <wrap/io_trimesh/export_obj.h>

#include <algorithm>
namespace flywave {
namespace voxelize {

template <typename Mesh> class xparam {
public:
  using mesh_type = Mesh;

  xparam(Mesh &_mesh, double tquality) : _curmesh(_mesh), _tquality(tquality) {}

  void parameter() {

    Mesh atlas_mesh;
    Mesh no_atlas_mesh;
    {
      size_t atlas_face_count = 0;
      size_t no_atlas_face_count = 0;

      for (auto &face : _curmesh.face) {
        if (face.tex == -1) {
          no_atlas_face_count++;
        } else {
          atlas_face_count++;
        }
      }

      vcg::tri::Allocator<Mesh>::AddFaces(atlas_mesh, atlas_face_count);
      vcg::tri::Allocator<Mesh>::AddFaces(no_atlas_mesh, no_atlas_face_count);

      vcg::tri::Allocator<Mesh>::AddVertices(atlas_mesh, atlas_face_count * 3);
      vcg::tri::Allocator<Mesh>::AddVertices(no_atlas_mesh,
                                             no_atlas_face_count * 3);

      size_t atlas_face_index = 0;
      size_t no_atlas_face_index = 0;
      for (auto &face : _curmesh.face) {
        if (face.tex != -1) {
          auto &v0 = atlas_mesh.vert[atlas_face_index * 3 + 0];
          v0 = *face.V(0);
          v0.node = face.node;
          v0.tmp_tex = face.tex;
          v0.tmp_mtl = face.mtl;

          auto &v1 = atlas_mesh.vert[atlas_face_index * 3 + 1];
          v1 = *face.V(1);
          v1.node = face.node;
          v1.tmp_tex = face.tex;
          v1.tmp_mtl = face.mtl;

          auto &v2 = atlas_mesh.vert[atlas_face_index * 3 + 2];
          v2 = *face.V(2);
          v2.node = face.node;
          v2.tmp_tex = face.tex;
          v2.tmp_mtl = face.mtl;

          atlas_mesh.face[atlas_face_index] = face;
          atlas_mesh.face[atlas_face_index].V(0) =
              &atlas_mesh.vert[atlas_face_index * 3 + 0];
          atlas_mesh.face[atlas_face_index].V(1) =
              &atlas_mesh.vert[atlas_face_index * 3 + 1];
          atlas_mesh.face[atlas_face_index].V(2) =
              &atlas_mesh.vert[atlas_face_index * 3 + 2];

          atlas_face_index++;
        } else {

          no_atlas_mesh.vert[no_atlas_face_index * 3 + 0] = *face.V(0);
          no_atlas_mesh.vert[no_atlas_face_index * 3 + 1] = *face.V(1);
          no_atlas_mesh.vert[no_atlas_face_index * 3 + 2] = *face.V(2);

          no_atlas_mesh.face[no_atlas_face_index] = face;
          no_atlas_mesh.face[no_atlas_face_index].V(0) =
              &no_atlas_mesh.vert[no_atlas_face_index * 3 + 0];
          no_atlas_mesh.face[no_atlas_face_index].V(1) =
              &no_atlas_mesh.vert[no_atlas_face_index * 3 + 1];
          no_atlas_mesh.face[no_atlas_face_index].V(2) =
              &no_atlas_mesh.vert[no_atlas_face_index * 3 + 2];

          no_atlas_face_index++;
        }
      }

      vcg::tri::Clean<Mesh>::RemoveDuplicateVertex(atlas_mesh);
      vcg::tri::Allocator<Mesh>::CompactVertexVector(atlas_mesh);
      vcg::tri::Allocator<Mesh>::CompactFaceVector(atlas_mesh);
      vcg::tri::UpdateNormal<Mesh>::PerVertex(atlas_mesh);

      vcg::tri::Clean<Mesh>::RemoveDuplicateVertex(no_atlas_mesh);
      vcg::tri::Allocator<Mesh>::CompactVertexVector(no_atlas_mesh);
      vcg::tri::Allocator<Mesh>::CompactFaceVector(no_atlas_mesh);
      vcg::tri::UpdateNormal<Mesh>::PerVertex(no_atlas_mesh);
    }

    xatlas::MeshDecl input_mesh;
    xatlas::Atlas *atlas = xatlas::Create();

    std::vector<float> vertexs(atlas_mesh.VN() * 3);
    {
      auto i = 0;
      for (auto &vertex : atlas_mesh.vert) {
        size_t index = i * 3;
        vertexs[index] = vertex.P()[0] * _tquality;
        vertexs[index + 1] = vertex.P()[1] * _tquality;
        vertexs[index + 2] = vertex.P()[2] * _tquality;
        i++;
      }

      input_mesh.vertexCount = atlas_mesh.VN();
      input_mesh.vertexPositionData = vertexs.data();
      input_mesh.vertexPositionStride = sizeof(float) * 3;
    }

    std::vector<float> normal(atlas_mesh.VN() * 3);
    {
      auto i = 0;
      for (auto &vertex : atlas_mesh.vert) {
        size_t index = i * 3;
        normal[index] = vertex.N()[0];
        normal[index + 1] = vertex.N()[1];
        normal[index + 2] = vertex.N()[2];
        i++;
      }
    }

    std::vector<uint32_t> faces(atlas_mesh.FN() * 3);
    {
      auto j = 0;
      for (auto &face : atlas_mesh.face) {
        size_t index = j * 3;
        faces[index] = vcg::tri::Index(atlas_mesh, face.V(0));
        faces[index + 1] = vcg::tri::Index(atlas_mesh, face.V(1));
        faces[index + 2] = vcg::tri::Index(atlas_mesh, face.V(2));
        j++;
      }

      input_mesh.indexCount = (uint32_t)atlas_mesh.FN()*3;
      input_mesh.indexData = faces.data();
      input_mesh.indexFormat = xatlas::IndexFormat::UInt32;
    }

    xatlas::AddMeshError::Enum error = xatlas::AddMesh(atlas, input_mesh, 1);

    // xatlas::AddMeshJoin(atlas);
    xatlas::Generate(atlas);

    {
      const xatlas::Mesh &output_mesh = atlas->meshes[0];
      _curmesh.Clear();
      size_t vertex_count = no_atlas_mesh.VN();
      size_t face_count = no_atlas_mesh.FN();

      vertex_count = output_mesh.vertexCount + no_atlas_mesh.VN();
      face_count = output_mesh.indexCount / 3 + no_atlas_mesh.FN();

      vcg::tri::Allocator<Mesh>::AddVertices(_curmesh, vertex_count);
      vcg::tri::Allocator<Mesh>::AddFaces(_curmesh, face_count);

      size_t vertex_index = 0;
      size_t face_index = 0;
      {
        for (auto &vertex : no_atlas_mesh.vert) {
          _curmesh.vert[vertex_index] = vertex;
          vertex_index++;
        }

        for (auto &face : no_atlas_mesh.face) {
          _curmesh.face[face_index] = face;
          _curmesh.face[face_index].V(0) =
              &_curmesh.vert[vcg::tri::Index(no_atlas_mesh, face.V(0))];
          _curmesh.face[face_index].V(1) =
              &_curmesh.vert[vcg::tri::Index(no_atlas_mesh, face.V(1))];
          _curmesh.face[face_index].V(2) =
              &_curmesh.vert[vcg::tri::Index(no_atlas_mesh, face.V(2))];

          face_index++;
        }
      }

      {

        _size.x = atlas->width;
        _size.y = atlas->height;

        auto nb_vert = output_mesh.vertexCount;
        auto nb_face = output_mesh.indexCount / 3;

        std::vector<typename Mesh::VertexPointer> ivp(nb_vert);
        for (int i = 0; i < nb_vert; ++i, ++vertex_index) {
          _curmesh.vert[vertex_index] =
              atlas_mesh.vert[output_mesh.vertexArray[i].xref];

          ivp[i] = &_curmesh.vert[vertex_index];
          _curmesh.vert[vertex_index].P() =
              atlas_mesh.vert[output_mesh.vertexArray[i].xref].P();
          _curmesh.vert[vertex_index].T().U() =
              output_mesh.vertexArray[i].uv[0];
          _curmesh.vert[vertex_index].T().V() =
              output_mesh.vertexArray[i].uv[1];
        }

        for (int i = 0; i < nb_face; ++i, ++face_index) {
          _curmesh.face[face_index].V(0) =
              ivp[output_mesh.indexArray[i * 3 + 0]];
          _curmesh.face[face_index].V(1) =
              ivp[output_mesh.indexArray[i * 3 + 1]];
          _curmesh.face[face_index].V(2) =
              ivp[output_mesh.indexArray[i * 3 + 2]];

          _curmesh.face[face_index].WT(0) = _curmesh.face[face_index].V(0)->T();
          _curmesh.face[face_index].WT(1) = _curmesh.face[face_index].V(1)->T();
          _curmesh.face[face_index].WT(2) = _curmesh.face[face_index].V(2)->T();

          _curmesh.face[face_index].node = _curmesh.face[face_index].V(0)->node;
          _curmesh.face[face_index].mtl =
              _curmesh.face[face_index].V(0)->tmp_mtl;
          _curmesh.face[face_index].tex =
              _curmesh.face[face_index].V(0)->tmp_tex;
        }

        xatlas::Destroy(atlas);
      }
    }
  }

  vector2<uint32_t> image_size() { return _size; }

  Mesh &mesh() { return _curmesh; }

private:
  Mesh &_curmesh;
  vector2<uint32_t> _size;
  double _tquality;
};

} // namespace voxelize
} // namespace flywave
