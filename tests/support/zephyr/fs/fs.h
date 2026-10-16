#pragma once
// Host boundary: the types ZephyrFS.h and ZephyrFsUtil.h name. No file system;
// the mesh-level zcpfs tests keep their few files in memory.
#include <cstdint>
struct fs_file_t { int unused; };
typedef uint8_t fs_mode_t;
inline void fs_file_t_init(struct fs_file_t *f) { f->unused = 0; }
