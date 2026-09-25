/*
 * SPDX-License-Identifier: MIT
 * ZephyrFsUtil - small filesystem helpers shared by every role's store.
 */

#include "ZephyrFsUtil.h"

#include <stdio.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zephcore_fs, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

bool zephcore_fs_is_mounted(const char *mount_point)
{
	/* fs_readmount() is a pure lookup. fs_statvfs() would log "mount point
	 * not found" at ERR for an absent /ext on every boot of a board without
	 * external flash, and before a deferred-init QSPI is mounted. */
	int index = 0;
	const char *name = NULL;

	while (fs_readmount(&index, &name) == 0) {
		if (name != NULL && strcmp(name, mount_point) == 0) {
			return true;
		}
	}
	return false;
}

bool zephcore_fs_exists(const char *path)
{
	struct fs_dirent ent;

	return fs_stat(path, &ent) == 0;
}

bool zephcore_fs_remove(const char *path)
{
	if (!zephcore_fs_exists(path)) {
		return true;
	}
	return fs_unlink(path) == 0;
}

bool zephcore_fs_read_file(const char *path, uint8_t *buf, size_t cap, size_t *out_len)
{
	struct fs_file_t file;

	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_READ) < 0) {
		return false;
	}
	ssize_t n = fs_read(&file, buf, cap);

	fs_close(&file);
	if (n < 0) {
		return false;
	}
	*out_len = (size_t)n;
	return true;
}

bool zephcore_fs_atomic_write(const char *path, zephcore_fs_writer_t writer, void *ctx,
			      const char *tag)
{
	char tmp_path[64];
	int pl = snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

	if (pl <= 0 || pl >= (int)sizeof(tmp_path)) {
		LOG_ERR("%s: path too long", tag);
		return false;
	}

	/* A temp left by an interrupted write */
	zephcore_fs_remove(tmp_path);

	struct fs_file_t file;

	fs_file_t_init(&file);
	int rc = fs_open(&file, tmp_path, FS_O_CREATE | FS_O_WRITE);

	if (rc < 0) {
		LOG_ERR("%s: open %s failed: %d", tag, tmp_path, rc);
		return false;
	}

	bool write_ok = writer(&file, ctx);
	int sync_rc = fs_sync(&file);

	fs_close(&file);
	if (!write_ok || sync_rc < 0) {
		if (!write_ok) {
			LOG_ERR("%s: write failed", tag);
		}
		if (sync_rc < 0) {
			LOG_ERR("%s: sync failed: %d", tag, sync_rc);
		}
		fs_unlink(tmp_path);
		return false;
	}

	rc = fs_rename(tmp_path, path);
	if (rc < 0) {
		LOG_ERR("%s: rename %s -> %s failed: %d", tag, tmp_path, path, rc);
		fs_unlink(tmp_path);
		return false;
	}
	return true;
}

struct replace_ctx {
	const uint8_t *buf;
	size_t len;
};

static bool replace_writer(struct fs_file_t *file, void *ctx)
{
	const struct replace_ctx *c = ctx;
	ssize_t n = fs_write(file, c->buf, c->len);

	return n >= 0 && (size_t)n == c->len;
}

bool zephcore_fs_atomic_replace(const char *path, const uint8_t *buf, size_t len,
				const char *tag)
{
	struct replace_ctx c = { .buf = buf, .len = len };

	return zephcore_fs_atomic_write(path, replace_writer, &c, tag);
}
