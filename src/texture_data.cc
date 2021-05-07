#include "texture_data.hh"

#include <memory>
#include <string.h>

#include "io_jpeg.hh"
#include "io_png.hh"

namespace flywave {

texture_data load_jpeg_image(std::vector<uint8_t> &&buf) {
  jpeg_uncompress_flags flag;
  int width;
  int height;
  int components;
  int64_t nwarn;

  std::unique_ptr<uint8_t[]> udata{jpeg_uncompress(
      buf.data(), buf.size(), flag, &width, &height, &components, &nwarn)};
  if (!udata)
    throw std::runtime_error("error");

  pixel_format format = pixel_format::RGB;

  if (components == 1) {
    format = pixel_format::R;
  } else if (components == 3) {
    format = pixel_format::RGB;
  }
  std::vector<uint8_t> dec_buf;
  dec_buf.resize(size_t(components * width * height * sizeof(uint8_t)), '\0');
  memcpy(&dec_buf[0], udata.get(), size_t(width * height * components));

  return texture_data{size_t(width), size_t(height), format, dec_buf};
}

texture_data load_png_image(std::vector<uint8_t> &&buf) {
  png_context decode;
  if (!png_init_decode(buf, 4, 8, &decode)) {
    throw std::runtime_error("error");
  }
  const int width = static_cast<int>(decode.width);
  const int height = static_cast<int>(decode.height);
  const int64_t total_size =
      static_cast<int64_t>(width) * static_cast<int64_t>(height);
  if (width != static_cast<int64_t>(decode.width) || width <= 0 ||
      width >= (1LL << 27) || height != static_cast<int64_t>(decode.height) ||
      height <= 0 || height >= (1LL << 27) || total_size >= (1LL << 29)) {
    png_free_decode(&decode);
    throw std::runtime_error("error");
  }
  std::vector<uint8_t> dec_buf;
  dec_buf.resize(size_t(decode.channels * width * height * sizeof(uint8_t)),
                 '\0');
  texture_data data{size_t(width), size_t(height), pixel_format::RGBA, dec_buf};

  if (!png_finish_decode(reinterpret_cast<png_bytep>(data.data.data()),
                         decode.channels * width * sizeof(uint8_t), &decode)) {
    png_free_decode(&decode);
    throw std::runtime_error("error");
  }

  png_free_decode(&decode);
  return data;
}

texture_data load_texture_data(std::vector<uint8_t> buff) {
  unsigned char ch1 = (unsigned char)buff[0];
  unsigned char ch2 = (unsigned char)buff[1];
  if (ch1 == 0x89 && ch2 == 0x50)
    return load_png_image(std::move(buff));
  else if (ch1 == 0xff && ch2 == 0xd8)
    return load_jpeg_image(std::move(buff));
  throw std::runtime_error("not support this image!");
}

} // namespace flywave
