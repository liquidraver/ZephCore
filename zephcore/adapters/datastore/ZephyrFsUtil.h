/*
 * SPDX-License-Identifier: MIT
 * ZephyrFsUtil - small filesystem helpers shared by every role's store.
 *
 * One implementation of the power-safe replace (temp file, sync, rename) and
 * of the probes that must not log. Zephyr's fs layer logs at ERR level for a
 * missing path (fs_open, fs_unlink) and an unknown mount point (fs_statvfs),
 * so every probe here checks first: an <err> line on the happy path of each
 * boot or save hides the real filesystem errors.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/fs/fs.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mount-list lookup; never logs. */
bool zephcore_fs_is_mounted(const char *mount_point);

bool zephcore_fs_exists(const char *path);

/* Idempotent: an absent file is a successful removal. */
bool zephcore_fs_remove(const char *path);

/* Read up to cap bytes of path. False if the file is absent or unreadable. */
bool zephcore_fs_read_file(const char *path, uint8_t *buf, size_t cap, size_t *out_len);

/* Write path through "<path>.tmp": writer fills the open temp file, then it is
 * synced and renamed over path. On any failure (writer false, sync, rename)
 * the temp is removed and path is left as it was. tag prefixes the log. */
typedef bool (*zephcore_fs_writer_t)(struct fs_file_t *file, void *ctx);
bool zephcore_fs_atomic_write(const char *path, zephcore_fs_writer_t writer, void *ctx,
			      const char *tag);

/* zephcore_fs_atomic_write() of one buffer; every byte must be written. */
bool zephcore_fs_atomic_replace(const char *path, const uint8_t *buf, size_t len,
				const char *tag);

#ifdef __cplusplus
}
#endif
