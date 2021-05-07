#pragma once

#include <cstdint>
#include <vector>

namespace flywave {

enum class pixel_format : uint16_t { R, RGB, RGBA };

struct texture_data {
  size_t width;
  size_t height;
  pixel_format format;
  std::vector<uint8_t> data;
};

texture_data load_texture_data(std::vector<uint8_t> buff);

} // namespace flywave
