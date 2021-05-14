#include "mesh_adapter.hh"
#include "voxel_mesh.hh"

namespace flywave {

void mesh_adapter::init_materials() {
  auto vm = dynamic_cast<voxel_mesh_adapter *>(_stream.get());
  auto bg = vm->_map.begin();
  while (bg != vm->_map.end()) {
    auto &mt = bg->second;
    auto md = std::make_shared<material_data>(*mt);
    md->_material_id = bg->first;
    if (mt->diffuse_texname.empty()) {
      material_group g{md};
      add_material(g);
    } else {
      auto st_p = std::make_shared<default_uv_policy>(
          std::make_unique<uv_reader_impl>(*vm->_mesh));
      auto &tx = vm->_tex_map.find(mt->diffuse_texname)->second;

      texture_sampler sampler{st_p, std::make_shared<color_extract_impl>(*tx)};
      material_group g{md, sampler};
      add_material(g);
    }
    bg++;
  }
}

} // namespace flywave