#include "io_jpeg.hh"

#include <algorithm>
#include <cassert>
#include <memory>
#include <setjmp.h>
#include <string.h>
#include <string>
#include <utility>

namespace flywave {
namespace jpeg {

void catch_error(j_common_ptr cinfo);

typedef struct {
  struct jpeg_destination_mgr pub;
  JOCTET *buffer;
  int bufsize;
  int datacount;
  std::vector<uint8_t> *dest;
} mem_dest_mgr;

typedef struct {
  struct jpeg_source_mgr pub;
  const unsigned char *data;
  unsigned long int datasize;
  bool try_recover_truncated_jpeg;
} mem_source_mgr;

void set_src(j_decompress_ptr cinfo, const void *data,
             unsigned long int datasize, bool try_recover_truncated_jpeg);

void set_sest(j_compress_ptr cinfo, void *buffer, int bufsize);

void set_sest(j_compress_ptr cinfo, void *buffer, int bufsize,
              std::vector<uint8_t> *destination);

void catch_error(j_common_ptr cinfo) {
  (*cinfo->err->output_message)(cinfo);
  jmp_buf *jpeg_jmpbuf = reinterpret_cast<jmp_buf *>(cinfo->client_data);
  jpeg_destroy(cinfo);
  longjmp(*jpeg_jmpbuf, 1);
}

void mem_init_destination(j_compress_ptr cinfo) {
  mem_dest_mgr *dest = reinterpret_cast<mem_dest_mgr *>(cinfo->dest);
  dest->pub.next_output_byte = dest->buffer;
  dest->pub.free_in_buffer = dest->bufsize;
  dest->datacount = 0;
  if (dest->dest) {
    dest->dest->clear();
  }
}

::boolean mem_empty_output_buffer(j_compress_ptr cinfo) {
  mem_dest_mgr *dest = reinterpret_cast<mem_dest_mgr *>(cinfo->dest);
  if (dest->dest) {
    dest->dest->resize(dest->dest->size() + dest->bufsize);
    memcpy(&(*dest->dest)[0], dest->buffer, dest->bufsize);
  }
  dest->pub.next_output_byte = dest->buffer;
  dest->pub.free_in_buffer = dest->bufsize;
  return TRUE;
}

void mem_term_destination(j_compress_ptr cinfo) {
  mem_dest_mgr *dest = reinterpret_cast<mem_dest_mgr *>(cinfo->dest);
  if (dest->dest) {
    dest->dest->resize(dest->dest->size() +
                       (dest->bufsize - dest->pub.free_in_buffer));
    memcpy(&(*dest->dest)[0], dest->buffer,
           (dest->bufsize - dest->pub.free_in_buffer));
  }
  dest->datacount = dest->bufsize - dest->pub.free_in_buffer;
}

void set_sest(j_compress_ptr cinfo, void *buffer, int bufsize) {
  set_sest(cinfo, buffer, bufsize, nullptr);
}

void set_sest(j_compress_ptr cinfo, void *buffer, int bufsize,
              std::vector<uint8_t> *destination) {
  mem_dest_mgr *dest;
  if (cinfo->dest == nullptr) {
    cinfo->dest = reinterpret_cast<struct jpeg_destination_mgr *>(
        (*cinfo->mem->alloc_small)(reinterpret_cast<j_common_ptr>(cinfo),
                                   JPOOL_PERMANENT, sizeof(mem_dest_mgr)));
  }

  dest = reinterpret_cast<mem_dest_mgr *>(cinfo->dest);
  dest->bufsize = bufsize;
  dest->buffer = static_cast<JOCTET *>(buffer);
  dest->dest = destination;
  dest->pub.init_destination = mem_init_destination;
  dest->pub.empty_output_buffer = mem_empty_output_buffer;
  dest->pub.term_destination = mem_term_destination;
}

void mem_init_source(j_decompress_ptr cinfo) {
  mem_source_mgr *src = reinterpret_cast<mem_source_mgr *>(cinfo->src);
  src->pub.next_input_byte = src->data;
  src->pub.bytes_in_buffer = src->datasize;
}

::boolean mem_fill_input_buffer(j_decompress_ptr cinfo) {
  static const JOCTET kEOIBuffer[2] = {0xff, JPEG_EOI};
  mem_source_mgr *src = reinterpret_cast<mem_source_mgr *>(cinfo->src);
  if (src->pub.bytes_in_buffer == 0 && src->pub.next_input_byte == src->data) {
    return FALSE;
  } else if (src->pub.bytes_in_buffer) {
    return src->try_recover_truncated_jpeg ? TRUE : FALSE;
  } else if (src->pub.next_input_byte != kEOIBuffer &&
             src->try_recover_truncated_jpeg) {
    src->pub.next_input_byte = kEOIBuffer;
    src->pub.bytes_in_buffer = 2;
    return TRUE;
  } else {
    return FALSE;
  }
}

void mem_term_source(j_decompress_ptr cinfo) {}

void mem_skip_input_data(j_decompress_ptr cinfo, long jump) {
  mem_source_mgr *src = reinterpret_cast<mem_source_mgr *>(cinfo->src);
  if (jump < 0) {
    return;
  }
  if (jump > src->pub.bytes_in_buffer) {
    src->pub.bytes_in_buffer = 0;
    (void)mem_fill_input_buffer(cinfo);
  } else {
    src->pub.bytes_in_buffer -= jump;
    src->pub.next_input_byte += jump;
  }
}

void set_src(j_decompress_ptr cinfo, const void *data,
             unsigned long int datasize, bool try_recover_truncated_jpeg) {
  mem_source_mgr *src;

  cinfo->src = reinterpret_cast<struct jpeg_source_mgr *>(
      (*cinfo->mem->alloc_small)(reinterpret_cast<j_common_ptr>(cinfo),
                                 JPOOL_PERMANENT, sizeof(mem_source_mgr)));

  src = reinterpret_cast<mem_source_mgr *>(cinfo->src);
  src->pub.init_source = mem_init_source;
  src->pub.fill_input_buffer = mem_fill_input_buffer;
  src->pub.skip_input_data = mem_skip_input_data;
  src->pub.resync_to_restart = jpeg_resync_to_restart;
  src->pub.term_source = mem_term_source;
  src->data = reinterpret_cast<const unsigned char *>(data);
  src->datasize = datasize;
  src->pub.bytes_in_buffer = 0;
  src->pub.next_input_byte = nullptr;
  src->try_recover_truncated_jpeg = try_recover_truncated_jpeg;
}

} // namespace jpeg

namespace {

enum jpeg_errors {
  JPEGERRORS_OK,
  JPEGERRORS_UNEXPECTED_END_OF_DATA,
  JPEGERRORS_BAD_PARAM
};

class fewer_args_for_compiler {
public:
  fewer_args_for_compiler(
      int datasize, const jpeg_uncompress_flags &flags, int64_t *nwarn,
      std::function<uint8_t *(int, int, int)> allocate_output)
      : datasize_(datasize), flags_(flags), pnwarn_(nwarn),
        allocate_output_(std::move(allocate_output)), height_read_(0),
        height_(0), stride_(0) {
    if (pnwarn_ != nullptr)
      *pnwarn_ = 0;
  }

  const int datasize_;
  const jpeg_uncompress_flags flags_;
  int64_t *const pnwarn_;
  std::function<uint8_t *(int, int, int)> allocate_output_;
  int height_read_;
  int height_;
  int stride_;
};

bool is_crop_window_valid(const jpeg_uncompress_flags &flags,
                          int input_image_width, int input_image_height) {
  return flags.crop_width > 0 && flags.crop_height > 0 && flags.crop_x >= 0 &&
         flags.crop_y >= 0 &&
         flags.crop_y + flags.crop_height <= input_image_height &&
         flags.crop_x + flags.crop_width <= input_image_width;
}

uint8_t *uncompress_low(const void *srcdata, fewer_args_for_compiler *argball) {
  const int datasize = argball->datasize_;
  const auto &flags = argball->flags_;
  const int ratio = flags.ratio;
  int components = flags.components;
  int stride = flags.stride;               // may be 0
  int64_t *const nwarn = argball->pnwarn_; // may be NULL

  if ((ratio != 1) && (ratio != 2) && (ratio != 4) && (ratio != 8)) {
    return nullptr;
  }

  if (!(components == 0 || components == 1 || components == 3)) {
    return nullptr;
  }

  if (datasize == 0 || srcdata == nullptr)
    return nullptr;

  JSAMPLE *tempdata = nullptr;

  jpeg_errors error = JPEGERRORS_OK;
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jmp_buf jpeg_jmpbuf;
  cinfo.client_data = &jpeg_jmpbuf;
  jerr.error_exit = jpeg::catch_error;
  if (setjmp(jpeg_jmpbuf)) {
    delete[] tempdata;
    return nullptr;
  }

  jpeg_create_decompress(&cinfo);
  jpeg::set_src(&cinfo, srcdata, datasize, flags.try_recover_truncated_jpeg);
  jpeg_read_header(&cinfo, TRUE);

  if (components == 0)
    components = std::min(cinfo.num_components, 3);

  switch (components) {
  case 1:
    cinfo.out_color_space = JCS_GRAYSCALE;
    break;
  case 3:
    if (cinfo.jpeg_color_space == JCS_CMYK ||
        cinfo.jpeg_color_space == JCS_YCCK) {
      cinfo.out_color_space = JCS_CMYK;
    } else {
      cinfo.out_color_space = JCS_RGB;
    }
    break;
  default:
    jpeg_destroy_decompress(&cinfo);
    return nullptr;
  }
  cinfo.do_fancy_upsampling = ::boolean(flags.fancy_upscaling);
  cinfo.scale_num = 1;
  cinfo.scale_denom = ratio;
  cinfo.dct_method = flags.dct_method;

  jpeg_start_decompress(&cinfo);

  int64_t total_size = static_cast<int64_t>(cinfo.output_height) *
                       static_cast<int64_t>(cinfo.output_width);

  if (cinfo.output_width <= 0 || cinfo.output_height <= 0) {
    jpeg_destroy_decompress(&cinfo);
    return nullptr;
  }
  if (total_size >= (1LL << 29)) {
    jpeg_destroy_decompress(&cinfo);
    return nullptr;
  }

  JDIMENSION target_output_width = cinfo.output_width;
  JDIMENSION target_output_height = cinfo.output_height;
  JDIMENSION skipped_scanlines = 0;

  const int min_stride = target_output_width * components * sizeof(JSAMPLE);
  if (stride == 0) {
    stride = min_stride;
  } else if (stride < min_stride) {
    jpeg_destroy_decompress(&cinfo);
    return nullptr;
  }

  argball->height_ = target_output_height;
  argball->stride_ = stride;

  uint8_t *dstdata = nullptr;
  if (flags.crop) {
    dstdata = new JSAMPLE[stride * target_output_height];
  } else {
    dstdata = argball->allocate_output_(target_output_width,
                                        target_output_height, components);
  }

  if (dstdata == nullptr) {
    jpeg_destroy_decompress(&cinfo);
    return nullptr;
  }
  JSAMPLE *output_line = static_cast<JSAMPLE *>(dstdata);

  const bool need_realign_cropped_scanline =
      (target_output_width != cinfo.output_width);
  const bool use_cmyk = (cinfo.out_color_space == JCS_CMYK);

  if (use_cmyk) {
    tempdata = new JSAMPLE[cinfo.output_width * 4];
  } else if (need_realign_cropped_scanline) {
    tempdata = new JSAMPLE[cinfo.output_width * components];
  }

  argball->height_read_ = target_output_height;

  const int max_scanlines_to_read = skipped_scanlines + target_output_height;
  const int mcu_align_offset =
      (cinfo.output_width - target_output_width) * (use_cmyk ? 4 : components);
  while (cinfo.output_scanline < max_scanlines_to_read) {
    int num_lines_read = 0;
    if (use_cmyk) {
      num_lines_read = jpeg_read_scanlines(&cinfo, &tempdata, 1);
      if (num_lines_read > 0) {
        for (size_t i = 0; i < target_output_width; ++i) {
          int offset = 4 * i;
          if (need_realign_cropped_scanline) {
            offset += mcu_align_offset;
          }
          const int c = tempdata[offset + 0];
          const int m = tempdata[offset + 1];
          const int y = tempdata[offset + 2];
          const int k = tempdata[offset + 3];
          int r, g, b;
          if (cinfo.saw_Adobe_marker) {
            r = (k * c) / 255;
            g = (k * m) / 255;
            b = (k * y) / 255;
          } else {
            r = (255 - k) * (255 - c) / 255;
            g = (255 - k) * (255 - m) / 255;
            b = (255 - k) * (255 - y) / 255;
          }
          output_line[3 * i + 0] = r;
          output_line[3 * i + 1] = g;
          output_line[3 * i + 2] = b;
        }
      }
    } else if (need_realign_cropped_scanline) {
      num_lines_read = jpeg_read_scanlines(&cinfo, &tempdata, 1);
      if (num_lines_read > 0) {
        memcpy(output_line, tempdata + mcu_align_offset, min_stride);
      }
    } else {
      num_lines_read = jpeg_read_scanlines(&cinfo, &output_line, 1);
    }
    if (num_lines_read == 0) {
      if (!flags.try_recover_truncated_jpeg) {
        argball->height_read_ = cinfo.output_scanline - skipped_scanlines;
        error = JPEGERRORS_UNEXPECTED_END_OF_DATA;
      } else {
        for (size_t line = cinfo.output_scanline; line < max_scanlines_to_read;
             ++line) {
          if (line == 0) {
            memset(output_line, 0, min_stride);
          } else {
            memcpy(output_line, output_line - stride, min_stride);
          }
          output_line += stride;
        }
        argball->height_read_ = target_output_height;
        cinfo.output_scanline = max_scanlines_to_read;
      }
      break;
    }
    output_line += stride;
  }
  delete[] tempdata;
  tempdata = nullptr;

  if (components == 4) {
    JSAMPLE *scanlineptr = static_cast<JSAMPLE *>(
        dstdata + static_cast<int64_t>(target_output_height - 1) * stride);
    const JSAMPLE kOpaque = -1; // All ones appropriate for JSAMPLE.
    const int right_rgb = (target_output_width - 1) * 3;
    const int right_rgba = (target_output_width - 1) * 4;

    for (int y = target_output_height; y-- > 0;) {
      const JSAMPLE *rgb_pixel = scanlineptr + right_rgb;
      JSAMPLE *rgba_pixel = scanlineptr + right_rgba;
      scanlineptr -= stride;
      for (int x = target_output_width; x-- > 0;
           rgba_pixel -= 4, rgb_pixel -= 3) {
        rgba_pixel[3] = kOpaque;
        rgba_pixel[2] = rgb_pixel[2];
        rgba_pixel[1] = rgb_pixel[1];
        rgba_pixel[0] = rgb_pixel[0];
      }
    }
  }

  switch (components) {
  case 1:
    if (cinfo.output_components != 1) {
      error = JPEGERRORS_BAD_PARAM;
    }
    break;
  case 3:
  case 4:
    if (cinfo.out_color_space == JCS_CMYK) {
      if (cinfo.output_components != 4) {
        error = JPEGERRORS_BAD_PARAM;
      }
    } else {
      if (cinfo.output_components != 3) {
        error = JPEGERRORS_BAD_PARAM;
      }
    }
    break;
  default:
    jpeg_destroy_decompress(&cinfo);
    return nullptr;
  }

  if (nwarn != nullptr) {
    *nwarn = cinfo.err->num_warnings;
  }

  switch (error) {
  case JPEGERRORS_OK:
    jpeg_finish_decompress(&cinfo);
    break;
  case JPEGERRORS_UNEXPECTED_END_OF_DATA:
  case JPEGERRORS_BAD_PARAM:
    jpeg_abort(reinterpret_cast<j_common_ptr>(&cinfo));
    break;
  default:
    break;
  }

  if (flags.crop) {
    target_output_height = flags.crop_height;
    target_output_width = flags.crop_width;

    if (!is_crop_window_valid(flags, cinfo.output_width, cinfo.output_height)) {
      delete[] dstdata;
      jpeg_destroy_decompress(&cinfo);
      return nullptr;
    }

    const uint8_t *full_image = dstdata;
    dstdata = argball->allocate_output_(target_output_width,
                                        target_output_height, components);
    if (dstdata == nullptr) {
      delete[] full_image;
      jpeg_destroy_decompress(&cinfo);
      return nullptr;
    }

    const int full_image_stride = stride;

    const int min_stride = target_output_width * components * sizeof(JSAMPLE);
    if (flags.stride == 0) {
      stride = min_stride;
    }
    argball->height_ = target_output_height;
    argball->stride_ = stride;

    if (argball->height_read_ > target_output_height) {
      argball->height_read_ = target_output_height;
    }
    const int crop_offset = flags.crop_x * components * sizeof(JSAMPLE);
    const uint8_t *full_image_ptr =
        full_image + flags.crop_y * full_image_stride;
    uint8_t *crop_image_ptr = dstdata;
    for (int i = 0; i < argball->height_read_; i++) {
      memcpy(crop_image_ptr, full_image_ptr + crop_offset, min_stride);
      crop_image_ptr += stride;
      full_image_ptr += full_image_stride;
    }
    delete[] full_image;
  }

  jpeg_destroy_decompress(&cinfo);
  return dstdata;
}

} // anonymous namespace

uint8_t *
jpeg_uncompress(const void *srcdata, int datasize,
                const jpeg_uncompress_flags &flags, int64_t *nwarn,
                std::function<uint8_t *(int, int, int)> allocate_output) {
  fewer_args_for_compiler argball(datasize, flags, nwarn,
                                  std::move(allocate_output));
  uint8_t *const dstdata = uncompress_low(srcdata, &argball);

  const float fraction_read =
      argball.height_ == 0
          ? 1.0
          : (static_cast<float>(argball.height_read_) / argball.height_);
  if (dstdata == nullptr ||
      fraction_read < std::min(1.0f, flags.min_acceptable_fraction)) {
    return nullptr;
  }

  if (argball.height_read_ != argball.height_) {
    const int first_bad_line = argball.height_read_;
    uint8_t *start = dstdata + first_bad_line * argball.stride_;
    const int nbytes = (argball.height_ - first_bad_line) * argball.stride_;
    memset(static_cast<void *>(start), 0, nbytes);
  }

  return dstdata;
}

uint8_t *jpeg_uncompress(const void *srcdata, int datasize,
                         const jpeg_uncompress_flags &flags, int *pwidth,
                         int *pheight, int *pcomponents, int64_t *nwarn) {
  uint8_t *buffer = nullptr;
  uint8_t *result =
      jpeg_uncompress(srcdata, datasize, flags, nwarn,
                      [=, &buffer](int width, int height, int components) {
                        if (pwidth != nullptr)
                          *pwidth = width;
                        if (pheight != nullptr)
                          *pheight = height;
                        if (pcomponents != nullptr)
                          *pcomponents = components;
                        buffer = new uint8_t[height * width * components];
                        return buffer;
                      });
  if (!result)
    delete[] buffer;
  return result;
}

bool jpeg_get_image_info(const void *srcdata, int datasize, int *width,
                         int *height, int *components) {
  if (width)
    *width = 0;
  if (height)
    *height = 0;
  if (components)
    *components = 0;

  if (datasize == 0 || srcdata == nullptr)
    return false;

  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  jmp_buf jpeg_jmpbuf;
  cinfo.err = jpeg_std_error(&jerr);
  cinfo.client_data = &jpeg_jmpbuf;
  jerr.error_exit = jpeg::catch_error;
  if (setjmp(jpeg_jmpbuf)) {
    return false;
  }

  jpeg_create_decompress(&cinfo);
  jpeg::set_src(&cinfo, srcdata, datasize, false);

  jpeg_read_header(&cinfo, TRUE);
  jpeg_start_decompress(&cinfo);
  if (width)
    *width = cinfo.output_width;
  if (height)
    *height = cinfo.output_height;
  if (components)
    *components = cinfo.output_components;

  jpeg_destroy_decompress(&cinfo);

  return true;
}

namespace {
bool compress_internal(const uint8_t *srcdata, int width, int height,
                       const jpeg_compress_flags &flags,
                       std::vector<uint8_t> *output) {
  output->clear();
  const int components = (static_cast<int>(flags.format) & 0xff);

  int64_t total_size =
      static_cast<int64_t>(width) * static_cast<int64_t>(height);

  if (width <= 0 || height <= 0) {
    return false;
  }
  if (total_size >= (1LL << 29)) {
    return false;
  }

  int in_stride = flags.stride;
  if (in_stride == 0) {
    in_stride = width * (static_cast<int>(flags.format) & 0xff);
  } else if (in_stride < width * components) {
    return false;
  }

  JOCTET *buffer = nullptr;

  assert(srcdata != nullptr);
  assert(output != nullptr);

  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  jmp_buf jpeg_jmpbuf;

  cinfo.err = jpeg_std_error(&jerr);
  cinfo.client_data = &jpeg_jmpbuf;
  jerr.error_exit = jpeg::catch_error;
  if (setjmp(jpeg_jmpbuf)) {
    output->clear();
    delete[] buffer;
    return false;
  }

  jpeg_create_compress(&cinfo);

  int bufsize = std::min(width * height * components, 1 << 20);
  buffer = new JOCTET[bufsize];
  jpeg::set_sest(&cinfo, buffer, bufsize, output);

  cinfo.image_width = width;
  cinfo.image_height = height;
  switch (components) {
  case 1:
    cinfo.input_components = 1;
    cinfo.in_color_space = JCS_GRAYSCALE;
    break;
  case 3:
  case 4:
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    break;
  default:
    output->clear();
    delete[] buffer;
    return false;
  }
  jpeg_set_defaults(&cinfo);
  if (flags.optimize_jpeg_size)
    cinfo.optimize_coding = TRUE;

  cinfo.density_unit = flags.density_unit; // JFIF code for pixel size units:
                                           // 1 = in, 2 = cm
  cinfo.X_density = flags.x_density;       // Horizontal pixel density
  cinfo.Y_density = flags.y_density;       // Vertical pixel density
  jpeg_set_quality(&cinfo, flags.quality, TRUE);

  if (flags.progressive) {
    jpeg_simple_progression(&cinfo);
  }

  if (!flags.chroma_downsampling) {
    for (int i = 0; i < cinfo.num_components; ++i) {
      cinfo.comp_info[i].h_samp_factor = 1;
      cinfo.comp_info[i].v_samp_factor = 1;
    }
  }

  jpeg_start_compress(&cinfo, TRUE);

  if (!flags.xmp_metadata.empty()) {
    const std::string name_space = "http://ns.adobe.com/xap/1.0/";
    const int name_space_length = name_space.size();
    const int metadata_length = flags.xmp_metadata.size();
    const int packet_length = metadata_length + name_space_length + 1;
    std::unique_ptr<JOCTET[]> joctet_packet(new JOCTET[packet_length]);

    for (int i = 0; i < name_space_length; i++) {
      joctet_packet[i] = name_space[i];
    }
    joctet_packet[name_space_length] = 0;

    for (int i = 0; i < metadata_length; i++) {
      joctet_packet[i + name_space_length + 1] = flags.xmp_metadata[i];
    }
    jpeg_write_marker(&cinfo, JPEG_APP0 + 1, joctet_packet.get(),
                      packet_length);
  }

  std::unique_ptr<JSAMPLE[]> row_temp(
      new JSAMPLE[width * cinfo.input_components]);
  while (cinfo.next_scanline < cinfo.image_height) {
    JSAMPROW row_pointer[1]; // pointer to JSAMPLE row[s]
    const uint8_t *r = &srcdata[cinfo.next_scanline * in_stride];
    uint8_t *p = static_cast<uint8_t *>(row_temp.get());
    switch (flags.format) {
    case FORMAT_RGBA: {
      for (int i = 0; i < width; ++i, p += 3, r += 4) {
        p[0] = r[0];
        p[1] = r[1];
        p[2] = r[2];
      }
      row_pointer[0] = row_temp.get();
      break;
    }
    case FORMAT_ABGR: {
      for (int i = 0; i < width; ++i, p += 3, r += 4) {
        p[0] = r[3];
        p[1] = r[2];
        p[2] = r[1];
      }
      row_pointer[0] = row_temp.get();
      break;
    }
    default: {
      row_pointer[0] = reinterpret_cast<JSAMPLE *>(const_cast<JSAMPLE *>(r));
    }
    }
    assert(jpeg_write_scanlines(&cinfo, row_pointer, 1) == 1u);
  }
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  delete[] buffer;
  return true;
}

} // anonymous namespace

bool jpeg_compress(const void *srcdata, int width, int height,
                   const jpeg_compress_flags &flags,
                   std::vector<uint8_t> *output) {
  return compress_internal(static_cast<const uint8_t *>(srcdata), width, height,
                           flags, output);
}

std::vector<uint8_t> jpeg_compress(const void *srcdata, int width, int height,
                                   const jpeg_compress_flags &flags) {
  std::vector<uint8_t> temp;
  compress_internal(static_cast<const uint8_t *>(srcdata), width, height, flags,
                    &temp);
  return temp;
}
} // namespace flywave
