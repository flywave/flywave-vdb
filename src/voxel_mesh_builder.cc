#include "voxel_mesh_builder.hh"

namespace flywave {

std::vector<triangle> voxel_mesh_builder::build_triangles() {
  std::vector<triangle> ts;
  for (auto &_data : _datas) {
    auto &mp = _data.mtl_faces_map();
    auto bg = mp.begin();
    while (bg != mp.end()) {
      auto &ver = _data.vertices();
      auto &fs = bg->second;
      auto &tx = _data.texcoords();
      for (uint32_t i = 0; i < fs.size(); i++) {
        triangle tri;
        tri.mtl_id = bg->first;
        auto &face = fs[i];
        tri.ver[0] = ver[face.x()];
        tri.ver[1] = ver[face.y()];
        tri.ver[2] = ver[face.z()];
        if (tx.size()) {
          tri.texcoord[0] = tx[face.x()];
          tri.texcoord[1] = tx[face.y()];
          tri.texcoord[2] = tx[face.z()];
        }
        ts.emplace_back(std::move(tri));
      }
      bg++;
    }
  }
  return ts;
}

int voxel_mesh_builder::add_material_data(material_data &&data, int index) {
  return add_material(
      std::make_shared<material>(std::forward<material_data>(data)), index);
}

int voxel_mesh_builder::add_material(std::shared_ptr<material> mt, int index) {
  if (index < 0) {
    index = _mtls.size();
  }
  if (mt->get_name().empty()) {
    std::stringstream ss;
    ss << "mtl" << index;
    mt->name = ss.str();
  }
  _mtls.emplace(index, std::move(mt));
  return index;
}

std::shared_ptr<voxel_mesh> voxel_mesh_builder::build_mesh() {
  auto hms = std::make_shared<voxel_mesh>();
  hms->add_tris(build_triangles());
  return hms;
}

} // namespace flywave
