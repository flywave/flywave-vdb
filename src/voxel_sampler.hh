#pragma once

#include "foundry.hh"
#include "simplify_mesh.hh"

#include <openvdb/openvdb.h>

#include <vector>

namespace flywave {

class texture_atlas_generate {
public:
  using texture_ptr = std::shared_ptr<texture2d<vdb::math::Vec4<uint8_t>>>;

  std::vector<texture_ptr> &generate(simplify_mesh &mesh,
                                     simplify_mesh &output_mesh);

  void push_foundry_atlas(std::shared_ptr<textute_foundry> foundry);

private:
  std::vector<std::shared_ptr<textute_foundry>> _foundries;
  std::vector<texture_ptr> _textures;
};

} // namespace flywave
