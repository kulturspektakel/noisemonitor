#pragma once

#include <stdbool.h>
#include <string.h>

static inline bool log_has_suffix(const char* filename, const char* suffix) {
  size_t filename_length = strlen(filename);
  size_t suffix_length = strlen(suffix);
  return filename_length >= suffix_length &&
         strcmp(filename + filename_length - suffix_length, suffix) == 0;
}
