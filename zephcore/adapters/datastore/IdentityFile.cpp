/*
 * SPDX-License-Identifier: MIT
 * IdentityFile - load and save a node identity file. See IdentityFile.h.
 */

#include "IdentityFile.h"
#include "ZephyrFsUtil.h"

#include <stdio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zephcore_identity, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

bool zephcore_identity_load(const char *path, mesh::LocalIdentity &id)
{
	if (!zephcore_fs_exists(path)) {
		return false;
	}

	uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE + 32];
	size_t len = 0;

	if (zephcore_fs_read_file(path, buf, sizeof(buf), &len) && len >= PRV_KEY_SIZE) {
		if (id.readFromStorage(buf, len)) {
			return true;
		}
		if (id.recoverFromStorage(buf, len)) {
			/* Deliberately not re-persisted: the file is the only record of
			 * what went wrong, and re-deriving costs a few ms per boot. */
			LOG_WRN("%s: pub/prv mismatch - advertising the pub its private key owns",
				path);
			return true;
		}
	}

	/* No layout coheres: adverts would verify nowhere, inbound DMs would not
	 * decrypt, and a key export would contradict SELF_INFO. */
	char bad_path[64];

	if (snprintf(bad_path, sizeof(bad_path), "%s.bad", path) < (int)sizeof(bad_path)) {
		zephcore_fs_remove(bad_path);
		if (fs_rename(path, bad_path) == 0) {
			LOG_ERR("%s: incoherent (%d bytes) - kept at %s, regenerating", path,
				(int)len, bad_path);
			return false;
		}
	}
	LOG_ERR("%s: incoherent (%d bytes) - could not park it, regenerating over it", path,
		(int)len);
	return false;
}

bool zephcore_identity_save(const char *path, const mesh::LocalIdentity &id)
{
	uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE + 32];
	size_t n = id.writeToStorage(buf, sizeof(buf));

	if (n == 0) {
		return false;
	}
	return zephcore_fs_atomic_replace(path, buf, n, "identity");
}
