#pragma once

#include "repacker.hh"
#include "texture_mesh.hh"

#include <openvdb/openvdb.h>

#include <vector>

namespace flywave {

class texture_atlas_generator {
public:
  using texture_ptr = std::shared_ptr<texture2d<vdb::math::Vec4<uint8_t>>>;

  std::vector<texture_ptr> &generate(texture_mesh &mesh,
                                     texture_mesh &output_mesh);

  void push_atlas(std::shared_ptr<textute_repacker> foundry);

private:
  std::vector<std::shared_ptr<textute_repacker>> _foundries;
  std::vector<texture_ptr> _textures;
};

} // namespace flywave
