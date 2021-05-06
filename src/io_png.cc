#include "io_png.hh"

#include <iostream>
#include <string.h>
#include <string>
#include <sys/types.h>
#include <utility>
#include <vector>
#include <zlib.h>
#include <cassert>

namespace flywave {

inline bool is_little_endian() {
  const union {
    uint8_t u8[2];
    uint16_t u16;
  } u = {{1, 0}};
  return u.u16 == 1;
}

inline bool is_big_endian() { return !is_little_endian(); }

namespace {

#define PTR_INC(type, ptr, del)                                                \
  (ptr = reinterpret_cast<type *>(reinterpret_cast<char *>(ptr) + (del)))
#define CPTR_INC(type, ptr, del)                                               \
  (ptr = reinterpret_cast<const type *>(reinterpret_cast<const char *>(ptr) +  \
                                        (del)))

static void convert_8_to_16(const uint8_t *p8, int num_comps, int p8_row_bytes,
                            int width, int height_in, uint16_t *p16,
                            int p16_row_bytes) {
  int64_t height = static_cast<int64_t>(height_in);

  width *= num_comps;
  CPTR_INC(uint8_t, p8,
           (height - 1) * p8_row_bytes + (width - 1) * sizeof(*p8));
  PTR_INC(uint16_t, p16,
          (height - 1) * p16_row_bytes + (width - 1) * sizeof(*p16));
  int bump8 = width * sizeof(*p8) - p8_row_bytes;
  int bump16 = width * sizeof(*p16) - p16_row_bytes;
  for (; height-- != 0;
       CPTR_INC(uint8_t, p8, bump8), PTR_INC(uint16_t, p16, bump16)) {
    for (int w = width; w-- != 0; --p8, --p16) {
      uint32_t pix = *p8;
      pix |= pix << 8;
      *p16 = static_cast<uint16_t>(pix);
    }
  }
}

#undef PTR_INC
#undef CPTR_INC

void error_handler(png_structp png_ptr, png_const_charp msg) {
  png_context *const ctx =
      reinterpret_cast<png_context *>(png_get_io_ptr(png_ptr));
  ctx->error_condition = true;
  longjmp(png_jmpbuf(png_ptr), 1);
}

void warning_handler(png_structp png_ptr, png_const_charp msg) {
  std::cerr << "PNG warning: " << msg << std::endl;
}

void string_reader(png_structp png_ptr, png_bytep data, png_size_t length) {
  png_context *const ctx =
      reinterpret_cast<png_context *>(png_get_io_ptr(png_ptr));
  if (static_cast<png_size_t>(ctx->data_left) < length) {
    memset(data, 0, length);
    png_error(png_ptr, "More bytes requested to read than available");
  } else {
    memcpy(data, ctx->data, length);
    ctx->data += length;
    ctx->data_left -= length;
  }
}

void string_writer(png_structp png_ptr, png_bytep data, png_size_t length) {
  std::string *const s =
      reinterpret_cast<std::string *>(png_get_io_ptr(png_ptr));
  s->append(reinterpret_cast<const char *>(data), length);
}

void string_writer_flush(png_structp png_ptr) {}

char *check_metadata_string(const std::string &s) {
  const char *const c_string = s.c_str();
  const size_t length = s.size();
  if (strlen(c_string) != length) {
    std::cerr << "Warning! Metadata contains \\0 character(s).";
  }
  return const_cast<char *>(c_string);
}

} // namespace

void png_free_decode(png_context *context) {
  if (context->png_ptr) {
    png_destroy_read_struct(&context->png_ptr,
                            context->info_ptr ? &context->info_ptr : nullptr,
                            nullptr);
    context->png_ptr = nullptr;
    context->info_ptr = nullptr;
  }
}

bool decode_header(std::vector<uint8_t> png_string, int *width, int *height,
                   int *components, int *channel_bit_depth,
                   std::vector<std::pair<std::string, std::string>> *metadata) {
  png_context context;

  constexpr int kDesiredNumChannels = 1;
  constexpr int kDesiredChannelBits = 16;
  if (!png_init_decode(png_string, kDesiredNumChannels, kDesiredChannelBits,
                       &context)) {
    return false;
  }
  assert(width != nullptr);
  *width = static_cast<int>(context.width);
  assert(height != nullptr);
  *height = static_cast<int>(context.height);
  if (components != nullptr) {
    switch (context.color_type) {
    case PNG_COLOR_TYPE_PALETTE:
      *components =
          (png_get_valid(context.png_ptr, context.info_ptr, PNG_INFO_tRNS)) ? 4
                                                                            : 3;
      break;
    case PNG_COLOR_TYPE_GRAY:
      *components = 1;
      break;
    case PNG_COLOR_TYPE_GRAY_ALPHA:
      *components = 2;
      break;
    case PNG_COLOR_TYPE_RGB:
      *components = 3;
      break;
    case PNG_COLOR_TYPE_RGB_ALPHA:
      *components = 4;
      break;
    default:
      *components = 0;
      break;
    }
  }
  if (channel_bit_depth != nullptr) {
    *channel_bit_depth = context.bit_depth;
  }
  if (metadata != nullptr) {
    metadata->clear();
    png_textp text_ptr = nullptr;
    int num_text = 0;
    png_get_text(context.png_ptr, context.info_ptr, &text_ptr, &num_text);
    for (int i = 0; i < num_text; i++) {
      const png_text &text = text_ptr[i];
      metadata->push_back(std::make_pair(text.key, text.text));
    }
  }
  png_free_decode(&context);
  return true;
}

bool png_init_decode(std::vector<uint8_t> &png_string, int desired_channels,
                     int desired_channel_bits, png_context *context) {
  assert(desired_channel_bits == 8 || desired_channel_bits == 16);
  assert(0 <= desired_channels && desired_channels <= 4);
  context->error_condition = false;
  context->channels = desired_channels;
  context->png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, context,
                                            error_handler, warning_handler);
  if (!context->png_ptr) {
    return false;
  }
  if (setjmp(png_jmpbuf(context->png_ptr))) {
    png_free_decode(context);
    return false;
  }
  context->info_ptr = png_create_info_struct(context->png_ptr);
  if (!context->info_ptr || context->error_condition) {
    png_free_decode(context);
    return false;
  }
  context->data = reinterpret_cast<const uint8_t *>(png_string.data());
  context->data_left = png_string.size();
  png_set_read_fn(context->png_ptr, context, string_reader);
  png_read_info(context->png_ptr, context->info_ptr);
  png_get_IHDR(context->png_ptr, context->info_ptr, &context->width,
               &context->height, &context->bit_depth, &context->color_type,
               nullptr, nullptr, nullptr);
  if (context->error_condition) {
    png_free_decode(context);
    return false;
  }
  if (context->width <= 0 || context->height <= 0) {
    png_free_decode(context);
    return false;
  }
  const bool has_tRNS =
      (png_get_valid(context->png_ptr, context->info_ptr, PNG_INFO_tRNS)) != 0;
  if (context->channels == 0) {
    if (context->color_type == PNG_COLOR_TYPE_PALETTE) {
      if (has_tRNS) {
        context->channels = 4; // RGB + A(tRNS)
      } else {
        context->channels = 3; // RGB
      }
    } else {
      context->channels = png_get_channels(context->png_ptr, context->info_ptr);
    }
  }
  const bool has_alpha = (context->color_type & PNG_COLOR_MASK_ALPHA) != 0;
  if ((context->channels & 1) == 0) { // We desire alpha
    if (has_alpha) {                  // There is alpha
    } else if (has_tRNS) {
      png_set_tRNS_to_alpha(context->png_ptr); // Convert transparency to alpha
    } else {
      png_set_add_alpha(context->png_ptr, (1 << context->bit_depth) - 1,
                        PNG_FILLER_AFTER);
    }
  } else {                                   // We don't want alpha
    if (has_alpha || has_tRNS) {             // There is alpha
      png_set_strip_alpha(context->png_ptr); // Strip alpha
    }
  }

  if (context->bit_depth > 8 && desired_channel_bits <= 8)
    png_set_strip_16(context->png_ptr);

  context->need_to_synthesize_16 =
      (context->bit_depth <= 8 && desired_channel_bits == 16);

  png_set_packing(context->png_ptr);
  context->num_passes = png_set_interlace_handling(context->png_ptr);

  if (desired_channel_bits > 8 && is_little_endian()) {
    png_set_swap(context->png_ptr);
  }

  if (context->color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_palette_to_rgb(context->png_ptr);

  const bool want_gray = (context->channels < 3);
  const bool is_gray = !(context->color_type & PNG_COLOR_MASK_COLOR);
  if (is_gray) {
    if (context->bit_depth < 8) {
      png_set_expand_gray_1_2_4_to_8(context->png_ptr);
    }
  }
  if (want_gray) {
    if (!is_gray)
      png_set_rgb_to_gray(context->png_ptr, 1, 0.299, 0.587); // 601, JPG
  } else {
    if (is_gray)
      png_set_gray_to_rgb(context->png_ptr); // Enable gray -> RGB conversion
  }

  png_read_update_info(context->png_ptr, context->info_ptr);
  return true;
}

bool png_finish_decode(png_bytep data, int row_bytes, png_context *context) {
  assert(data != nullptr);

  if (setjmp(png_jmpbuf(context->png_ptr))) {
    png_free_decode(context);
    return false;
  }

  for (int p = 0; p < context->num_passes; ++p) {
    png_bytep row = data;
    for (int h = context->height; h-- != 0; row += row_bytes) {
      png_read_row(context->png_ptr, row, nullptr);
    }
  }

  png_set_rows(context->png_ptr, context->info_ptr,
               png_get_rows(context->png_ptr, context->info_ptr));
  png_read_end(context->png_ptr, context->info_ptr);

  const bool ok = !context->error_condition;
  png_free_decode(context);

  if (context->need_to_synthesize_16)
    convert_8_to_16(reinterpret_cast<uint8_t *>(data), context->channels,
                    row_bytes, context->width, context->height,
                    reinterpret_cast<uint16_t *>(data), row_bytes);
  return ok;
}

bool png_image_to_buffer(
    const void *image, int width, int height, int row_bytes, int num_channels,
    int channel_bits, int compression, std::vector<uint8_t> *png_string,
    const std::vector<std::pair<std::string, std::string>> *metadata) {
  assert(image != nullptr);
  assert(png_string != nullptr);

  if (width == 0 || height == 0)
    return false;

  png_string->resize(0);
  png_infop info_ptr = nullptr;
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr,
                                                error_handler, warning_handler);
  if (png_ptr == nullptr)
    return false;
  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_write_struct(&png_ptr, info_ptr ? &info_ptr : nullptr);
    return false;
  }
  info_ptr = png_create_info_struct(png_ptr);
  if (info_ptr == nullptr) {
    png_destroy_write_struct(&png_ptr, nullptr);
    return false;
  }

  int color_type = -1;
  switch (num_channels) {
  case 1:
    color_type = PNG_COLOR_TYPE_GRAY;
    break;
  case 2:
    color_type = PNG_COLOR_TYPE_GRAY_ALPHA;
    break;
  case 3:
    color_type = PNG_COLOR_TYPE_RGB;
    break;
  case 4:
    color_type = PNG_COLOR_TYPE_RGB_ALPHA;
    break;
  default:
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return false;
  }

  png_set_write_fn(png_ptr, png_string, string_writer, string_writer_flush);
  if (compression < 0)
    compression = Z_DEFAULT_COMPRESSION;
  png_set_compression_level(png_ptr, compression);
  png_set_compression_mem_level(png_ptr, MAX_MEM_LEVEL);

  png_set_IHDR(png_ptr, info_ptr, width, height, channel_bits, color_type,
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
               PNG_FILTER_TYPE_DEFAULT);

  if (metadata && !metadata->empty()) {
    std::vector<png_text> text;
    for (const auto &pair : *metadata) {
      png_text txt;
      txt.compression = PNG_TEXT_COMPRESSION_NONE;
      txt.key = check_metadata_string(pair.first);
      txt.text = check_metadata_string(pair.second);
      text.push_back(txt);
    }
    png_set_text(png_ptr, info_ptr, &text[0], text.size());
  }

  png_write_info(png_ptr, info_ptr);
  if (channel_bits > 8 && is_little_endian())
    png_set_swap(png_ptr);

  png_byte *row = reinterpret_cast<png_byte *>(const_cast<void *>(image));
  for (; height--; row += row_bytes)
    png_write_row(png_ptr, row);
  png_write_end(png_ptr, nullptr);
  png_destroy_write_struct(&png_ptr, &info_ptr);
  return true;
}
} // namespace flywave
