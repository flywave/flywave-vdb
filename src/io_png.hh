#pragma once

#include <string>
#include <utility>
#include <vector>

extern "C" {
#include <png.h>
}

namespace flywave {

struct png_context {
  const uint8_t *data;
  int data_left;
  png_structp png_ptr;
  png_infop info_ptr;
  png_uint_32 width, height;
  int num_passes;
  int color_type;
  int bit_depth;
  int channels;
  bool need_to_synthesize_16;
  bool error_condition;

  png_context() : png_ptr(nullptr), info_ptr(nullptr) {}
};

bool decode_header(std::vector<uint8_t> png_string, int *width, int *height,
                   int *components, int *channel_bit_depth,
                   std::vector<std::pair<std::string, std::string>> *metadata);

bool png_init_decode(std::vector<uint8_t> &png_string, int desired_channels,
                     int desired_channel_bits, png_context *context);

bool png_finish_decode(png_bytep data, int row_bytes, png_context *context);

void png_free_decode(png_context *context);

bool png_image_to_buffer(
    const void *image, int width, int height, int row_bytes, int num_channels,
    int channel_bits, int compression, std::vector<uint8_t> *png_string,
    const std::vector<std::pair<std::string, std::string>> *metadata);

} // namespace flywave
