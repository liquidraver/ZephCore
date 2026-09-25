/*
 * SPDX-License-Identifier: MIT
 * IdentityFile - load and save a node identity file (pub || prv, as Arduino
 * MeshCore's IdentityStore), shared by every role's store.
 */

#pragma once

#include <mesh/Identity.h>

/* False if the file is absent, or holds no coherent key pair. In the second
 * case the bytes are first moved to "<path>.bad", because the caller then
 * generates a fresh identity and saves it over path, and the old private key
 * may still be recoverable by hand. A stored pub that does not match its prv
 * is repaired in RAM (the pub the private key owns) and reported each boot. */
bool zephcore_identity_load(const char *path, mesh::LocalIdentity &id);

/* Power-safe replace (ZephyrFsUtil). */
bool zephcore_identity_save(const char *path, const mesh::LocalIdentity &id);
