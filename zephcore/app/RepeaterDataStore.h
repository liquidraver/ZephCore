/*
 * SPDX-License-Identifier: MIT
 * RepeaterDataStore - Filesystem storage for repeater
 *
 * Uses /lfs/repeater/ prefix to keep data separate from companion, so the
 * two prefs layouts (311 B here, 272 B there, different field order; see
 * helpers/PrefsCodec.h) can never be read through each other.
 *
 * The roles are NOT interchangeable: booting a repeater onto a companion
 * volume formats it, and vice versa.  They are different kinds of node, and
 * they share one 128 KB LittleFS volume - a companion's contacts and blob
 * cache would crowd out repeater writes (-ENOSPC), not corrupt them.  Save
 * your identity before switching roles.
 */

#pragma once

#include <cstdint>
#include <stddef.h>
#include <mesh/Identity.h>
#include <helpers/NodePrefs.h>
#include <helpers/ClientACL.h>
#include <helpers/RegionMap.h>

class RepeaterDataStore {
public:
	RepeaterDataStore();

	/* Initialize filesystem and repeater directory */
	bool begin();

	/* Identity management */
	bool loadIdentity(mesh::LocalIdentity& id);
	bool saveIdentity(const mesh::LocalIdentity& id);

	/* Prefs management */
	bool loadPrefs(NodePrefs& prefs);
	bool savePrefs(const NodePrefs& prefs);

	/* The /lfs/repeater directory as a FILESYSTEM, for the storage code ported
	 * from upstream (ClientACL, RegionMap), which names files from its root. */
	FILESYSTEM* getFS() { return &_fs; }

	/* Factory reset - erase all repeater data */
	bool formatFileSystem();

	/* True if this LittleFS volume already holds THIS role's data.  False
	 * means the volume belongs to something else - a fresh chip, a companion,
	 * or a node that was running Arduino MeshCore, whose nRF52 filesystems
	 * overlap our lfs_partition (devdocs/HANDOVER_lfs_arduino_overlap.md).
	 * Callers format on false; see the note on BASE_PATH below. */
	bool hasRoleData() const;

	/* Get base path for repeater storage */
	const char* getBasePath() const;

private:
	bool _initialized;
	/* Legacy binary prefs (PrefsCodec): read once to migrate, then kept for
	 * firmware from before prefs.json. */
	bool loadLegacyPrefs(const char* path, NodePrefs& prefs);
	static constexpr const char* BASE_PATH = "/lfs/repeater";
	ZephyrFS _fs{BASE_PATH};
};
