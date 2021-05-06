#pragma once

#include <functional>
#include <string>
#include <vector>

extern "C" {
#include <jpeglib.h>
}

namespace flywave {

struct jpeg_uncompress_flags {
  int ratio = 1;

  int components = 0;

  bool fancy_upscaling = true;

  bool try_recover_truncated_jpeg = false;

  float min_acceptable_fraction = 1.0;

  int stride = 0;

  J_DCT_METHOD dct_method = JDCT_DEFAULT;

  bool crop = false;

  int crop_x = 0;

  int crop_y = 0;

  int crop_width = 0;

  int crop_height = 0;
};

uint8_t *jpeg_uncompress(const void *srcdata, int datasize,
                         const jpeg_uncompress_flags &flags, int *width,
                         int *height, int *components, int64_t *nwarn);

uint8_t *
jpeg_uncompress(const void *srcdata, int datasize,
                const jpeg_uncompress_flags &flags, int64_t *nwarn,
                std::function<uint8_t *(int, int, int)> allocate_output);

bool jpeg_get_image_info(const void *srcdata, int datasize, int *width,
                         int *height, int *components);

enum jpeg_format {
  FORMAT_GRAYSCALE = 0x001, // 1 byte/pixel
  FORMAT_RGB = 0x003,       // 3 bytes/pixel RGBRGBRGBRGB...
  FORMAT_RGBA = 0x004,      // 4 bytes/pixel RGBARGBARGBARGBA...
  FORMAT_ABGR = 0x104       // 4 bytes/pixel ABGRABGRABGR...
};

struct jpeg_compress_flags {
  jpeg_format format;

  int quality = 95;

  bool progressive = false;

  bool optimize_jpeg_size = false;

  bool chroma_downsampling = true;

  int density_unit = 1; // 1 = in, 2 = cm

  int x_density = 300;
  int y_density = 300;

  std::string xmp_metadata;

  int stride = 0;
};

std::vector<uint8_t> jpeg_compress(const void *srcdata, int width, int height,
                                   const jpeg_compress_flags &flags);

bool jpeg_compress(const void *srcdata, int width, int height,
                   const jpeg_compress_flags &flags,
                   std::vector<uint8_t> *output);

} // namespace flywave
