#pragma once

#include <functional>
#include <list>
#include <queue>
#include <vector>
#include <memory>
#include <string>

namespace flywave {

enum texture_type {
  TEXTURE_TYPE_UNKNOWN,
  TEXTURE_TYPE_UINT8,
  TEXTURE_TYPE_UINT16,
  TEXTURE_TYPE_FLOAT,
  TEXTURE_TYPE_DOUBLE
};

class base_texture : public std::enable_shared_from_this<base_texture> {
public:
  std::unique_ptr<uint8_t[]> data;

  base_texture() = default;

  virtual ~base_texture() {}

  virtual bool valid() const = 0;

  virtual size_t color_count() const = 0;

  virtual size_t channels() const = 0;

  virtual size_t dim() const = 0;

  virtual size_t stride() const = 0;

  virtual size_t bytes() const = 0;

  inline uint8_t *raw_data() { return data.get(); }

  inline const uint8_t *raw_data() const { return data.get(); }

  virtual texture_type get_type() const { return TEXTURE_TYPE_UNKNOWN; }

  virtual char const *get_type_string() const { return "unknown"; }

  static texture_type get_type_for_string(std::string const &type_string) {
    if (type_string == "uint8")
      return TEXTURE_TYPE_UINT8;
    else if (type_string == "uint16")
      return TEXTURE_TYPE_UINT16;
    else if (type_string == "float")
      return TEXTURE_TYPE_FLOAT;
    else if (type_string == "double")
      return TEXTURE_TYPE_DOUBLE;

    return TEXTURE_TYPE_UNKNOWN;
  }

protected:
  base_texture(std::unique_ptr<uint8_t[]> d) : data(std::move(d)) {}
};

} // namespace flywave
