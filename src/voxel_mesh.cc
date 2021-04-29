#include "voxel_mesh.hh"
#include "st_policy.hh"

namespace flywave {

voxel_mesh_adapter::voxel_mesh_adapter(
    voxel_mesh &&hm, std::unordered_map<int, material> &map,
    std::unordered_map<std::string, texture> &tex_map)
    : _map(map), _tex_map(tex_map), _mesh(hm) {}

void voxel_mesh_adapter::fill_meterial(std::shared_ptr<mesh_adapter> ada) {
  auto bg = _map.begin();
  while (bg != _map.end()) {
    auto &mt = bg->second;
    auto md = std::make_shared<material_data>(mt);
    md->_material_id = bg->first;
    if (mt.diffuse_texname.empty()) {
      material_group g{md};
      ada->add_material(g);
    } else {
      auto st_p = std::make_shared<uv_st_policy>(
          std::make_unique<uv_reader_impl>(_mesh));
      auto &tx = _tex_map.find(mt.diffuse_texname)->second;

      texture_sampler sampler{st_p, std::make_shared<color_extract_impl>(tx)};
      material_group g{md, sampler};
      ada->add_material(g);
    }
    bg++;
  }
}

material::operator material_data() const {
  material_data md;
  md.type = type;
  md.color.x() = uint8_t(color.x() * 255);
  md.color.y() = uint8_t(color.y() * 255);
  md.color.z() = uint8_t(color.z() * 255);

  md.ambient.x() = uint8_t(ambient.x() * 255);
  md.ambient.y() = uint8_t(ambient.y() * 255);
  md.ambient.z() = uint8_t(ambient.z() * 255);

  md.emissive.x() = uint8_t(emissive.x() * 255);
  md.emissive.y() = uint8_t(emissive.y() * 255);
  md.emissive.z() = uint8_t(emissive.z() * 255);

  md.specular.x() = uint8_t(specular.x() * 255);
  md.specular.y() = uint8_t(specular.y() * 255);
  md.specular.z() = uint8_t(specular.z() * 255);

  md.opacity = transparency;
  md.shininess = shiness;
  md.metallic = metallic;
  md.roughness = roughness;
  md.reflectance = reflectance;
  md.clearcoat_thickness = clearcoat_thickness;
  md.clearcoat_roughness = clearcoat_roughness;
  md.anisotropy = anisotropy;
  md.anisotropy_rotation = anisotropy_rotation;
  md.mode = mode;

  return md;
}

color_type color_extract_impl::extract(const material_group &fgroup,
                                       const vdb::Vec2d &uv,
                                       const triangle3<double> &tri) {
  if (std::isnan(uv.x()) || std::isnan(uv.y()))
    return color_type{0, 0, 0, 0};
  float uvx = uv.x();
  if (uvx > 1)
    uvx -= std::floor(uvx);
  if (uvx < 0)
    uvx += std::ceil(std::abs(uvx));

  float uvy = uv.y();
  if (uvy > 1)
    uvy -= std::floor(uvy);
  if (uvy < 0)
    uvy += std::ceil(std::abs(uvy));

  auto x = (_tx.data->width() - 1) * uvx;
  auto y = (_tx.data->height() - 1) * (1 - uvy);

  int xc = std::ceil(x);
  int yc = std::ceil(y);

  return _tx.get(xc, yc);
}

triangle2<double> uv_reader_impl::get(uint32_t index) {
  auto tri = _hm.find_triangle(index);
  return triangle2<double>{tri.texcoord[0], tri.texcoord[1], tri.texcoord[2]};
}
} // namespace flywave