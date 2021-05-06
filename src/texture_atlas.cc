#include "texture_atlas.hh"

namespace flywave {

template <typename Param> class packed_mesh : public Param {
public:
  packed_mesh(typename Param::mesh_type &mesh, float quality)
      : Param(mesh, quality) {}

public:
  texture2d<vdb::math::Vec4<uint8_t>>::Ptr
  pack_texture(std::shared_ptr<textute_repacker> tfound) {
    auto img_size = Param::image_size();
    if (img_size.first == 0 || img_size.second == 0)
      return nullptr;

    texture2d<vdb::math::Vec4<uint8_t>>::Ptr _texture =
        texture2d<vdb::math::Vec4<uint8_t>>::create(
            {img_size.first, img_size.second});

    _texture->fill('\0');
    int count = 0;
    for (auto &fptr : Param::mesh().face) {
      count++;
      if (fptr.tex == -1)
        continue;

      auto image = tfound->extract(
          fmesh_tri_patch{
              openvdb::Vec3d(fptr.P(0)[0], fptr.P(0)[1], fptr.P(0)[2]),
              openvdb::Vec3d(fptr.P(1)[0], fptr.P(1)[1], fptr.P(1)[2]),
              openvdb::Vec3d(fptr.P(2)[0], fptr.P(2)[1], fptr.P(2)[2]),
              openvdb::Vec2d(fptr.WT(0).U(), fptr.WT(0).V()),
              openvdb::Vec2d(fptr.WT(1).U(), fptr.WT(1).V()),
              openvdb::Vec2d(fptr.WT(2).U(), fptr.WT(2).V())},
          _texture);
    }
    double e = 1.0 / (img_size.first - 1);
    double e1 = 1.0 / (img_size.second - 1);
    for (auto &fptr : Param::mesh().face) {
      fptr.V(0)->T().U() = fptr.WT(0).U() * e;
      fptr.V(0)->T().V() = fptr.WT(0).V() * e1;

      fptr.V(1)->T().U() = fptr.WT(1).U() * e;
      fptr.V(1)->T().V() = fptr.WT(1).V() * e1;

      fptr.V(2)->T().U() = fptr.WT(2).U() * e;
      fptr.V(2)->T().V() = fptr.WT(2).V() * e1;

      fptr.WT(0).U() = fptr.WT(0).U() * e;
      fptr.WT(0).V() = fptr.WT(0).V() * e1;

      fptr.WT(1).U() = fptr.WT(1).U() * e;
      fptr.WT(1).V() = fptr.WT(1).V() * e1;

      fptr.WT(2).U() = fptr.WT(2).U() * e;
      fptr.WT(2).V() = fptr.WT(2).V() * e1;
    }
    return _texture;
  }
};

std::vector<texture_atlas_generator::texture_ptr> &
texture_atlas_generator::generate(texture_mesh &_texture_mesh,
                                  texture_mesh &output_mesh) {
  int current_texture_id = -1;
  _textures.clear();
  std::unordered_map<int, std::vector<texture_face *>> texture_index_map;
  std::vector<texture_face *> *_count_ptr = nullptr;
  for (auto &face : _texture_mesh.face) {
    if (face.tex == -1) {
      continue;
    }
    if (current_texture_id != face.tex) {
      current_texture_id = face.tex;
      _count_ptr = &texture_index_map[current_texture_id];
    }
    _count_ptr->emplace_back(&face);
  }

  std::unordered_map<int, texture_mesh> texture_meshies;

  for (auto &iter : texture_index_map) {
    vcg::tri::Allocator<texture_mesh>::AddFaces(texture_meshies[iter.first],
                                                iter.second.size());
    vcg::tri::Allocator<texture_mesh>::AddVertices(texture_meshies[iter.first],
                                                   iter.second.size() * 3);
    int face_index = 0;
    for (auto &face : iter.second) {
      auto &tar_face = texture_meshies[iter.first].face[face_index];

      auto &vert0 = texture_meshies[iter.first].vert[face_index * 3 + 0];
      vert0 = *face->V(0);

      auto &vert1 = texture_meshies[iter.first].vert[face_index * 3 + 1];
      vert1 = *face->V(1);

      auto &vert2 = texture_meshies[iter.first].vert[face_index * 3 + 2];
      vert2 = *face->V(2);

      tar_face.V(0) = &vert0;
      tar_face.V(1) = &vert1;
      tar_face.V(2) = &vert2;

      tar_face.tex = face->tex;
      tar_face.node = face->node;
      tar_face.mtl = face->mtl;
      tar_face.feature_id = face->feature_id;

      face_index++;
    }

    vcg::tri::Clean<texture_mesh>::RemoveDuplicateVertex(
        texture_meshies[iter.first]);
    vcg::tri::Allocator<texture_mesh>::CompactVertexVector(
        texture_meshies[iter.first]);
    vcg::tri::Allocator<texture_mesh>::CompactFaceVector(
        texture_meshies[iter.first]);
    vcg::tri::UpdateNormal<texture_mesh>::PerVertex(
        texture_meshies[iter.first]);
  }

  auto sum_face_size = 0;
  for (auto &iter : texture_index_map) {
    auto packed = std::make_shared<packed_mesh<xparam<texture_mesh>>>(
        texture_meshies[iter.first], 1);
    packed->parameter();
    _textures.emplace_back(packed->pack_texture(_foundries[iter.first]));
    sum_face_size += texture_meshies[iter.first].face.size();
  }

  {
    output_mesh.Clear();
    vcg::tri::Allocator<texture_mesh>::AddFaces(output_mesh, sum_face_size);
    vcg::tri::Allocator<texture_mesh>::AddVertices(output_mesh,
                                                   sum_face_size * 3);

    int face_index = 0;
    for (auto &iter : texture_meshies) {
      for (auto &face : texture_meshies[iter.first].face) {
        auto &tar_face = output_mesh.face[face_index];

        auto &vert0 = output_mesh.vert[face_index * 3 + 0];
        vert0 = *face.V(0);

        auto &vert1 = output_mesh.vert[face_index * 3 + 1];
        vert1 = *face.V(1);

        auto &vert2 = output_mesh.vert[face_index * 3 + 2];
        vert2 = *face.V(2);

        tar_face.V(0) = &vert0;
        tar_face.V(1) = &vert1;
        tar_face.V(2) = &vert2;

        tar_face.WT(0) = face.WT(0);
        tar_face.WT(1) = face.WT(1);
        tar_face.WT(2) = face.WT(2);

        tar_face.tex = face.tex;
        tar_face.node = face.node;
        tar_face.mtl = face.mtl;
        tar_face.feature_id = face.feature_id;

        face_index++;
      }
    }

    vcg::tri::Clean<texture_mesh>::RemoveDuplicateVertex(output_mesh);
    vcg::tri::Allocator<texture_mesh>::CompactVertexVector(output_mesh);
    vcg::tri::Allocator<texture_mesh>::CompactFaceVector(output_mesh);
    vcg::tri::UpdateNormal<texture_mesh>::PerVertex(output_mesh);
  }

  return _textures;
}

void texture_atlas_generator::push_atlas(
    std::shared_ptr<textute_repacker> foundry) {
  _foundries.push_back(foundry);
}

} // namespace flywave
