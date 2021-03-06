#include "material_data.hh"

namespace flywave {

bool material_data::operator==(const material_data &p) const {
  if (type == p.type) {
    switch (type) {
    case BASE:
      return mode == p.mode && color == p.color &&
             vdb::math::isApproxEqual(opacity, p.opacity);
    case LAMBERT:
      return mode == p.mode && color == p.color &&
             vdb::math::isApproxEqual(opacity, p.opacity) &&
             ambient == p.ambient && emissive == p.emissive;
    case PHONG:
      return mode == p.mode && color == p.color &&
             vdb::math::isApproxEqual(opacity, p.opacity) &&
             ambient == p.ambient && emissive == p.emissive &&
             specular == p.specular &&
             vdb::math::isApproxEqual(shininess, p.shininess);
    case PBR:
      return mode == p.mode && color == p.color &&
             vdb::math::isApproxEqual(opacity, p.opacity) &&
             vdb::math::isApproxEqual(metallic, p.metallic) &&
             vdb::math::isApproxEqual(roughness, p.roughness) &&
             vdb::math::isApproxEqual(reflectance, p.reflectance) &&
             vdb::math::isApproxEqual(clearcoat_thickness,
                                      p.clearcoat_thickness) &&
             vdb::math::isApproxEqual(clearcoat_roughness,
                                      p.clearcoat_roughness) &&
             vdb::math::isApproxEqual(anisotropy, p.anisotropy) &&
             vdb::math::isApproxEqual(anisotropy_rotation,
                                      p.anisotropy_rotation);
    default:
      break;
    }
  }
  return false;
}

void material_data::read(std::istream &is) {
  is >> _material_id >> type >> mode >> color.x() >> color.y() >> color.z() >>
      ambient.x() >> ambient.y() >> ambient.z() >> emissive.x() >>
      emissive.y() >> emissive.z() >> specular.x() >> specular.y() >>
      specular.z() >> opacity >> shininess >> metallic >> roughness >>
      reflectance >> clearcoat_thickness >> clearcoat_roughness >> anisotropy >>
      anisotropy_rotation;
}

void material_data::write(std::ostream &os) const {
  os << _material_id << type << mode << color.x() << color.y() << color.z()
     << ambient.x() << ambient.y() << ambient.z() << emissive.x()
     << emissive.y() << emissive.z() << specular.x() << specular.y()
     << specular.z() << opacity << shininess << metallic << roughness
     << reflectance << clearcoat_thickness << clearcoat_roughness << anisotropy
     << anisotropy_rotation;
}
} // namespace flywave

std::ostream &operator<<(std::ostream &os, const flywave::material_data &m) {
  os << "Material{ id : " << m._material_id << ", type :" << m.type
     << ", mode :" << m.mode << ", color :[" << m.color.x() << ","
     << m.color.y() << "," << m.color.z() << "]"
     << ", ambient :[" << m.ambient.x() << "," << m.ambient.y() << ","
     << m.ambient.z() << "], emissive :[" << m.emissive.x() << ","
     << m.emissive.y() << m.emissive.z() << ", specular :[" << m.specular.x()
     << "," << m.specular.y() << "," << m.specular.z()
     << ", opacity :" << m.opacity << ", shininess :" << m.shininess
     << ", metallic :" << m.metallic << ", roughness :" << m.roughness
     << ", reflectance :" << m.reflectance
     << ", clearcoat_thickness :" << m.clearcoat_thickness
     << ", clearcoat_roughness :" << m.clearcoat_roughness
     << ", anisotropy :" << m.anisotropy
     << ", anisotropy_rotation :" << m.anisotropy_rotation << "}";
  return os;
}
