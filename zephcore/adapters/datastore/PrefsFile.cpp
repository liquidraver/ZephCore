/*
 * SPDX-License-Identifier: MIT
 * PrefsFile - read and write a prefs.json file. See PrefsFile.h.
 */

#include "PrefsFile.h"
#include "FsStream.h"
#include "ZephyrFsUtil.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zephcore_prefs_file, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

bool zephcore_prefs_json_load(const char *path, NodePrefs &p, PrefsFromJsonFn from_json)
{
	struct fs_dirent ent;

	if (fs_stat(path, &ent) != 0) {
		return false;
	}
	if (ent.size == 0) {
		/* The parser accepts an empty stream as "no keys" */
		LOG_ERR("%s is empty", path);
		return false;
	}

	struct fs_file_t file;

	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_READ) < 0) {
		LOG_ERR("%s: open failed", path);
		return false;
	}
	FsReadStream in(&file);
	bool ok = from_json(p, in);

	fs_close(&file);
	if (!ok) {
		LOG_ERR("%s does not parse (%u bytes)", path, (unsigned)ent.size);
	}
	return ok;
}

struct json_writer_ctx {
	const NodePrefs *p;
	PrefsToJsonFn to_json;
};

static bool json_writer(struct fs_file_t *file, void *arg)
{
	const struct json_writer_ctx *c = static_cast<const struct json_writer_ctx *>(arg);
	FsWriteStream out(file);

	return c->to_json(*c->p, out) && out.ok();
}

/* True if serialising p would reproduce the file byte for byte. Most
 * savePrefs() callers save whatever the command touched, including a value
 * set to what it already was; this keeps those off the flash. */
static bool prefs_json_unchanged(const char *path, const NodePrefs &p, PrefsToJsonFn to_json)
{
	if (!zephcore_fs_exists(path)) {
		return false;
	}

	struct fs_file_t file;

	fs_file_t_init(&file);
	if (fs_open(&file, path, FS_O_READ) < 0) {
		return false;
	}
	FsCompareStream cmp(&file);
	bool same = to_json(p, cmp) && cmp.same();

	fs_close(&file);
	return same;
}

bool zephcore_prefs_json_save(const char *path, const NodePrefs &p, PrefsToJsonFn to_json)
{
	if (prefs_json_unchanged(path, p, to_json)) {
		LOG_DBG("%s unchanged, not rewritten", path);
		return true;
	}

	struct json_writer_ctx c = { &p, to_json };

	return zephcore_fs_atomic_write(path, json_writer, &c, "savePrefs");
}
