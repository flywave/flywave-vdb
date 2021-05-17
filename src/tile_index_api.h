#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(WIN32) || defined(WINDOWS) || defined(_WIN32) || defined(_WINDOWS)
#define FLYWAVE_TILE_INDEX_API __declspec(dllexport)
#else
#define FLYWAVE_TILE_INDEX_API
#endif

extern FLYWAVE_TILE_INDEX_API uint32_t tile_index_path_level(uint64_t path_);

extern FLYWAVE_TILE_INDEX_API bool tile_index_path_is_valid(uint64_t path_);

extern FLYWAVE_TILE_INDEX_API uint32_t tile_index_which_child(uint64_t path_);

extern FLYWAVE_TILE_INDEX_API uint32_t
tile_index_level_bits_at_pos(uint64_t path_, uint32_t position);

extern FLYWAVE_TILE_INDEX_API uint64_t
tile_index_new_from_level_row_col(uint32_t level, uint32_t row, uint32_t col);

extern FLYWAVE_TILE_INDEX_API uint64_t
tile_index_new_from_branchlist(uint32_t level, const unsigned char blist[]);

extern FLYWAVE_TILE_INDEX_API uint64_t
tile_index_new_from_other(uint64_t other, uint32_t level);

extern FLYWAVE_TILE_INDEX_API uint64_t
tile_index_new_get_generation_sequence(uint64_t path_);

extern FLYWAVE_TILE_INDEX_API void tile_index_get_level_row_col(uint64_t path_,
                                                                uint32_t *level,
                                                                uint32_t *row,
                                                                uint32_t *col);

extern FLYWAVE_TILE_INDEX_API _Bool tile_index_less(uint64_t path_,
                                                    uint64_t other);

extern FLYWAVE_TILE_INDEX_API _Bool
tile_index_advance_in_level(uint64_t *path_);

extern FLYWAVE_TILE_INDEX_API uint64_t tile_index_get_child(uint64_t path_,
                                                            uint32_t child);

extern FLYWAVE_TILE_INDEX_API uint64_t tile_index_get_parent(uint64_t path_);

extern FLYWAVE_TILE_INDEX_API _Bool tile_index_advance(uint64_t *path_,
                                                       uint32_t max_level);

extern FLYWAVE_TILE_INDEX_API _Bool tile_index_is_ancestor_of(uint64_t path_,
                                                              uint64_t other);

extern FLYWAVE_TILE_INDEX_API uint64_t tile_index_relative_path(uint64_t parent,
                                                                uint64_t child);

extern FLYWAVE_TILE_INDEX_API uint64_t
tile_index_concatenate(uint64_t path_, uint64_t sub_path);

extern FLYWAVE_TILE_INDEX_API uint64_t tile_index_as_index(uint64_t path_,
                                                           uint32_t level);

#ifdef __cplusplus
}
#endif
