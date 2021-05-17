#include "tile_index_api.h"

#include <algorithm>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

const uint32_t kMaxLevel = 24;
const uint32_t kStoreSize = sizeof(uint64_t);
const uint32_t kChildCount = 4;
static const uint32_t kLevelBits = 2; // bits per level in packed path_
static const uint64_t kLevelBitMask = 0x03;
static const uint32_t kTotalBits = 64; // total storage bits
static const uint64_t kPathMask = ~(~uint64_t(0) >> (kMaxLevel * kLevelBits));
static const uint64_t kLevelMask = ~kPathMask;

inline uint64_t path_bits(uint64_t path_) { return path_ & kPathMask; }
inline uint64_t path_mask(uint32_t level) {
  return kPathMask << ((kMaxLevel - level) * kLevelBits);
}

inline uint64_t path_bits_level(uint64_t path_, uint32_t level) {
  return path_ & path_mask(level);
}

inline uint32_t path_level(uint64_t path_) { return path_ & kLevelMask; }

FLYWAVE_TILE_INDEX_API uint32_t tile_index_path_level(uint64_t path_) {
  return path_ & kLevelMask;
}

FLYWAVE_TILE_INDEX_API bool
tile_index_path_is_valid(uint64_t path_) { // level in range, no stray bits
  return path_level(path_) <= kMaxLevel &&
         (0 == (path_ & ~(path_mask(path_level(path_)) | kLevelMask)));
}

FLYWAVE_TILE_INDEX_API uint32_t tile_index_which_child(uint64_t path_) {
  return (path_ >> (kTotalBits - path_level(path_) * kLevelBits)) &
         kLevelBitMask;
}

FLYWAVE_TILE_INDEX_API uint32_t
tile_index_level_bits_at_pos(uint64_t path_, uint32_t position) {
  return (path_ >> (kTotalBits - (position + 1) * kLevelBits)) & kLevelBitMask;
}

FLYWAVE_TILE_INDEX_API uint64_t
tile_index_new_from_level_row_col(uint32_t level, uint32_t row, uint32_t col) {
  static const uint64_t order[][2] = {{0, 3}, {1, 2}};

  assert(level <= kMaxLevel);
  uint64_t path_ = 0;

  for (uint32_t j = 0; j < level; ++j) {
    uint32_t right = 0x01 & (col >> (level - j - 1));
    uint32_t top = 0x01 & (row >> (level - j - 1));
    path_ |= order[right][top] << (kTotalBits - ((j + 1) * kLevelBits));
  }

  path_ |= level;
  return path_;
}

FLYWAVE_TILE_INDEX_API uint64_t
tile_index_new_from_branchlist(uint32_t level, const unsigned char blist[]) {
  assert(level <= kMaxLevel);
  uint64_t path_ = 0;

  for (uint32_t j = 0; j < level; ++j) {
    path_ |= (blist[j] & kLevelBitMask)
             << (kTotalBits - ((j + 1) * kLevelBits));
  }
  path_ |= level;
  return path_;
}

FLYWAVE_TILE_INDEX_API uint64_t tile_index_new_from_other(uint64_t other,
                                                          uint32_t level) {
  uint32_t lev = std::min(level, path_level(other));
  return path_bits_level(other, lev) | lev;
}

FLYWAVE_TILE_INDEX_API uint64_t
tile_index_new_get_generation_sequence(uint64_t path_) {
  const uint32_t level = path_level(path_);
  uint64_t sequence = path_;
  uint64_t check_for_2_or_3_mask = ((uint64_t)0x1) << (kTotalBits - 1);
  uint64_t interchange_2_or_3_mask = ((uint64_t)0x01) << (kTotalBits - 2);

  for (uint32_t j = 0; j < level;
       ++j, check_for_2_or_3_mask >>= 2, interchange_2_or_3_mask >>= 2) {
    if (sequence & check_for_2_or_3_mask) {
      sequence ^= interchange_2_or_3_mask;
    }
  }
  return sequence;
}

FLYWAVE_TILE_INDEX_API void tile_index_get_level_row_col(uint64_t path_,
                                                         uint32_t *level,
                                                         uint32_t *row,
                                                         uint32_t *col) {
  static const uint32_t rowbits[] = {0x00, 0x00, 0x01, 0x01};
  static const uint32_t colbits[] = {0x00, 0x01, 0x01, 0x00};

  uint32_t row_val = 0;
  uint32_t col_val = 0;

  for (uint32_t j = 0; j < path_level(path_); ++j) {
    uint32_t level_bits = tile_index_level_bits_at_pos(path_, j);
    row_val = (row_val << 1) | (rowbits[level_bits]);
    col_val = (col_val << 1) | (colbits[level_bits]);
  }

  *level = path_level(path_);
  *row = row_val;
  *col = col_val;
}

FLYWAVE_TILE_INDEX_API _Bool tile_index_less(uint64_t path_, uint64_t other) {
  uint32_t minlev = (path_level(path_) < path_level(other)) ? path_level(path_)
                                                            : path_level(other);

  uint64_t mask = ~(~uint64_t(0) >> (minlev * kLevelBits));
  if (mask & (path_ ^ other)) {
    return path_bits(path_) < path_bits(other);
  } else {
    return path_level(path_) < path_level(other);
  }
}

FLYWAVE_TILE_INDEX_API _Bool tile_index_advance_in_level(uint64_t *path_) {
  uint64_t path_bits_ = path_bits(*path_);
  uint64_t path_mask_ = path_mask(path_level(*path_));
  if (path_bits_ != path_mask_) {
    *path_ += uint64_t(1) << (kTotalBits - path_level(*path_) * kLevelBits);
    assert(tile_index_path_is_valid(*path_));
    return true;
  } else {
    return false;
  }
}

FLYWAVE_TILE_INDEX_API uint64_t tile_index_get_child(uint64_t path_,
                                                     uint32_t child) {
  assert(path_level(path_) <= kMaxLevel);
  assert(child <= 3);
  uint32_t new_level = path_level(path_) + 1;
  return uint64_t(path_bits(path_) |
                  uint64_t(child) << (kTotalBits - new_level * kLevelBits) |
                  new_level);
}

FLYWAVE_TILE_INDEX_API uint64_t tile_index_get_parent(uint64_t path_) {
  assert(path_level(path_) > 0);
  uint32_t new_level = path_level(path_) - 1;

  return uint64_t(
      (path_ & (kPathMask << kLevelBits * (kMaxLevel - new_level))) |
      new_level);
}

FLYWAVE_TILE_INDEX_API _Bool tile_index_advance(uint64_t *path_,
                                                uint32_t max_level) {
  assert(max_level > 0);
  assert(path_level(*path_) <= max_level);
  if (path_level(*path_) < max_level) {
    *path_ = tile_index_get_child(*path_, 0);
    return true;
  } else {
    while (tile_index_which_child(*path_) == kChildCount - 1) {
      *path_ = tile_index_get_parent(*path_);
    }
    return tile_index_advance_in_level(path_);
  }
}

FLYWAVE_TILE_INDEX_API _Bool tile_index_is_ancestor_of(uint64_t path_,
                                                       uint64_t other) {
  if (path_level(path_) <= path_level(other)) {
    return path_bits_level(path_, path_level(path_)) ==
           path_bits_level(other, path_level(path_));
  } else {
    return false;
  }
}

FLYWAVE_TILE_INDEX_API uint64_t tile_index_relative_path(uint64_t parent,
                                                         uint64_t child) {
  assert(tile_index_is_ancestor_of(parent, child));
  unsigned int levelDiff = path_level(child) - path_level(parent);
  return uint64_t((path_bits(child) << (path_level(parent) * kLevelBits)) |
                  levelDiff);
}

FLYWAVE_TILE_INDEX_API uint64_t tile_index_concatenate(uint64_t path_,
                                                       uint64_t sub_path) {
  uint64_t level = path_level(path_) + path_level(sub_path);
  assert(level <= kMaxLevel);
  return uint64_t((path_ & kPathMask) |
                  ((sub_path & kPathMask) >> path_level(path_) * kLevelBits) |
                  level);
}

FLYWAVE_TILE_INDEX_API uint64_t tile_index_as_index(uint64_t path_,
                                                    uint32_t level) {
  return (path_ >> (kTotalBits - level * kLevelBits));
}

#ifdef __cplusplus
}
#endif
