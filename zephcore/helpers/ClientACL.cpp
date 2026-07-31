/*
 * SPDX-License-Identifier: MIT
 * ClientACL - Access Control List for repeater clients
 */

#include "ClientACL.h"
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zephcore_acl, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

void ClientACL::load(const char* path, const mesh::LocalIdentity& self_id) {
    _path = path;
    num_clients = 0;

    struct fs_file_t file;
    fs_file_t_init(&file);

    if (fs_open(&file, path, FS_O_READ) < 0) {
        LOG_DBG("No ACL file at %s", path);
        return;
    }

    bool full = false;
    while (!full) {
        ClientInfo c;
        uint8_t pub_key[32];
        uint8_t unused[2];

        c.clear();

        bool success = (fs_read(&file, pub_key, 32) == 32);
        success = success && (fs_read(&file, &c.permissions, 1) == 1);
        success = success && (fs_read(&file, &c.extra.room.sync_since, 4) == 4);
        success = success && (fs_read(&file, unused, 2) == 2);
        success = success && (fs_read(&file, &c.out_path_len, 1) == 1);
        success = success && (fs_read(&file, c.out_path, 64) == 64);
        success = success && (fs_read(&file, c.shared_secret, PUB_KEY_SIZE) == PUB_KEY_SIZE);

        if (!success) break;  // EOF

        c.id = mesh::Identity(pub_key);
        self_id.calcSharedSecret(c.shared_secret, pub_key);  // recalculate in case key changed

        if (num_clients < MAX_CLIENTS) {
            clients[num_clients++] = c;
        } else {
            full = true;
        }
    }
    fs_close(&file);
    LOG_INF("Loaded %d clients from %s", num_clients, path);
}

void ClientACL::save(const char* path, bool (*filter)(ClientInfo*)) {
    _path = path;

    // Remove old file first (LittleFS doesn't overwrite well)
    fs_unlink(path);

    struct fs_file_t file;
    fs_file_t_init(&file);

    if (fs_open(&file, path, FS_O_CREATE | FS_O_WRITE) < 0) {
        LOG_ERR("Failed to open %s for write", path);
        return;
    }

    uint8_t unused[2] = {0, 0};
    int saved = 0;

    for (int i = 0; i < num_clients; i++) {
        auto c = &clients[i];
        if (c->permissions == 0 || (filter && !filter(c))) continue;  // skip deleted or filtered

        bool success = (fs_write(&file, c->id.pub_key, 32) == 32);
        success = success && (fs_write(&file, &c->permissions, 1) == 1);
        success = success && (fs_write(&file, &c->extra.room.sync_since, 4) == 4);
        success = success && (fs_write(&file, unused, 2) == 2);
        success = success && (fs_write(&file, &c->out_path_len, 1) == 1);
        success = success && (fs_write(&file, c->out_path, 64) == 64);
        success = success && (fs_write(&file, c->shared_secret, PUB_KEY_SIZE) == PUB_KEY_SIZE);

        if (!success) {
            LOG_ERR("Write failed at client %d", i);
            break;
        }
        saved++;
    }
    fs_close(&file);
    LOG_INF("Saved %d clients to %s", saved, path);
}

bool ClientACL::clear() {
    if (_path) {
        fs_unlink(_path);
    }
    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients[i].clear();
    }
    num_clients = 0;
    return true;
}

ClientInfo* ClientACL::getClient(const uint8_t* pubkey, int key_len) {
    for (int i = 0; i < num_clients; i++) {
        if (memcmp(pubkey, clients[i].id.pub_key, key_len) == 0) {
            return &clients[i];  // found
        }
    }
    return nullptr;  // not found
}

ClientInfo* ClientACL::putClient(const mesh::Identity& id, uint8_t init_perms) {
    uint32_t min_time = 0xFFFFFFFF;
    ClientInfo* oldest = &clients[MAX_CLIENTS - 1];

    for (int i = 0; i < num_clients; i++) {
        if (id.matches(clients[i].id)) {
            return &clients[i];  // already known
        }
        if (!clients[i].isAdmin() && clients[i].last_activity < min_time) {
            oldest = &clients[i];
            min_time = oldest->last_activity;
        }
    }

    ClientInfo* c;
    if (num_clients < MAX_CLIENTS) {
        c = &clients[num_clients++];
    } else {
        c = oldest;  // evict least active contact
    }
    c->clear();
    c->permissions = init_perms;
    c->id = id;
    c->out_path_len = OUT_PATH_UNKNOWN;  // initially out_path is unknown
    return c;
}

bool ClientACL::applyPermissions(const mesh::LocalIdentity& self_id, const uint8_t* pubkey, int key_len, uint8_t perms) {
    ClientInfo* c;
    if ((perms & PERM_ACL_ROLE_MASK) == PERM_ACL_GUEST) {
        // guest role is not persisted in contacts
        c = getClient(pubkey, key_len);
        if (c == nullptr) return false;  // partial pubkey not found

        num_clients--;  // delete from contacts[]
        int i = c - clients;
        while (i < num_clients) {
            clients[i] = clients[i + 1];
            i++;
        }
    } else {
        if (key_len < PUB_KEY_SIZE) return false;  // need complete pubkey when adding/modifying

        mesh::Identity id(pubkey);
        c = putClient(id, 0);

        c->permissions = perms;  // update their permissions
        self_id.calcSharedSecret(c->shared_secret, pubkey);
    }
    return true;
}
