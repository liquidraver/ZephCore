/*
 * SPDX-License-Identifier: MIT
 */

#include "observer_creds.h"

#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(zephcore_observer_creds, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

extern "C" bool observer_creds_load(struct ObserverCreds *creds,
				    const char *base_path)
{
	char path[96];
	snprintf(path, sizeof(path), "%s/obs_creds", base_path);

	struct fs_file_t f;
	fs_file_t_init(&f);

	int rc = fs_open(&f, path, FS_O_READ);
	if (rc < 0) {
		LOG_DBG("obs_creds not found (%d), using defaults", rc);
		memset(creds, 0, sizeof(*creds));
		observer_creds_init(creds);
		return false;
	}

	ssize_t n = fs_read(&f, creds, sizeof(*creds));
	fs_close(&f);

	if (n != (ssize_t)sizeof(*creds)) {
		LOG_WRN("obs_creds truncated (%d/%d), resetting", (int)n,
			(int)sizeof(*creds));
		memset(creds, 0, sizeof(*creds));
		observer_creds_init(creds);
		return false;
	}

	return true;
}

extern "C" bool observer_creds_save(const struct ObserverCreds *creds,
				    const char *base_path)
{
	/* Atomic replace (.tmp + fs_sync + fs_rename), same pattern as
	 * identity/prefs — power loss mid-write must not corrupt the creds. */
	char path[96];
	char tmp_path[104];
	snprintf(path, sizeof(path), "%s/obs_creds", base_path);
	if (snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path) >=
	    (int)sizeof(tmp_path)) {
		return false;
	}

	fs_unlink(tmp_path);

	struct fs_file_t f;
	fs_file_t_init(&f);

	int rc = fs_open(&f, tmp_path, FS_O_WRITE | FS_O_CREATE | FS_O_TRUNC);
	if (rc < 0) {
		LOG_ERR("Cannot open %s for write: %d", tmp_path, rc);
		return false;
	}

	ssize_t n = fs_write(&f, creds, sizeof(*creds));
	rc = fs_sync(&f);
	fs_close(&f);

	if (n != (ssize_t)sizeof(*creds) || rc < 0) {
		LOG_ERR("obs_creds write failed (%d/%d sync=%d)", (int)n,
			(int)sizeof(*creds), rc);
		fs_unlink(tmp_path);
		return false;
	}

	if (fs_rename(tmp_path, path) < 0) {
		LOG_ERR("obs_creds rename failed");
		fs_unlink(tmp_path);
		return false;
	}

	return true;
}
