#pragma once

#include "types.hh"

#include <openvdb/openvdb.h>

#include <unordered_map>
#include <vector>

namespace flywave {

struct material_data {
  material_id_t _material_id = -1;

  enum { BASE = 0, LAMBERT = 1, PHONG = 2, PBR = 3 };
  enum { COLOR = 0x1, TEXTURE = 0x2, BUMP = 0x4 };

  uint32_t type = 0;
  uint16_t mode{0};

  vdb::math::Vec3<uint8_t> color{255, 255, 255};

  vdb::math::Vec3<uint8_t> ambient{255, 255, 255};
  vdb::math::Vec3<uint8_t> emissive{0, 0, 0};

  vdb::math::Vec3<uint8_t> specular{0, 0, 0};

  float opacity{1.0};

  float shininess{20.0f};

  float metallic{0.0f};
  float roughness{1.0f};
  float reflectance{0.5f};

  float clearcoat_thickness{0.0f};
  float clearcoat_roughness{0.0f};

  float anisotropy{0.f};
  float anisotropy_rotation{0.0f};

  bool operator==(const material_data &p) const;

  explicit operator bool() const { return _material_id != material_id_t(-1); }

  void read(std::istream &is);

  void write(std::ostream &os) const;
};

} // namespace flywave

std::ostream &operator<<(std::ostream &, const flywave::material_data &);

namespace openvdb {
namespace OPENVDB_VERSION_NAME {

template <> inline const char *typeNameAsString<flywave::material_data>() {
  return "material";
}

template <> inline flywave::material_data zeroVal<flywave::material_data>() {
  return flywave::material_data();
}
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb
