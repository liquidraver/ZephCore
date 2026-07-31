/*
 * SPDX-License-Identifier: MIT
 * RepeaterDataStore - Filesystem storage for repeater
 *
 * Uses /lfs/repeater/ prefix to keep data separate from companion.
 * This allows flashing back and forth between roles without corruption.
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

    /* ACL management - paths passed to ClientACL */
    const char* getAclPath() const;

    /* Region management - paths passed to RegionMap */
    const char* getRegionsPath() const;

    /* Factory reset - erase all repeater data */
    bool formatFileSystem();

    /* Get base path for repeater storage */
    const char* getBasePath() const;

private:
    bool _initialized;
    static constexpr const char* BASE_PATH = "/lfs/repeater";
};
