/*
 * SPDX-License-Identifier: Apache-2.0
 * RegionMap - Region-based flood filtering for repeaters
 */

#include "RegionMap.h"
#include <helpers/TxtDataHelpers.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(zephcore_regions, CONFIG_ZEPHCORE_DATASTORE_LOG_LEVEL);

static const char* skip_hash(const char* name) {
    return *name == '#' ? name + 1 : name;
}

RegionMap::RegionMap(TransportKeyStore& store) : _store(&store) {
    next_id = 1;
    num_regions = 0;
    default_id = home_id = 0;
    wildcard.id = 0;
    wildcard.parent = 0;
    wildcard.flags = 0;  // default behaviour, allow flood and direct
    strcpy(wildcard.name, "*");
}

bool RegionMap::is_name_char(uint8_t c) {
    // accept all alpha-num or accented characters, but exclude most punctuation chars
    return c == '-' || c == '$' || c == '#' || (c >= '0' && c <= '9') || c >= 'A';
}

bool RegionMap::load(const char* path) {
    const char* filepath = path;

    struct fs_file_t file;
    fs_file_t_init(&file);

    if (fs_open(&file, filepath, FS_O_READ) < 0) {
        LOG_DBG("No regions file at %s", filepath);
        return false;
    }

    uint8_t pad[128];
    num_regions = 0;
    next_id = 1;
    default_id = home_id = 0;

    bool success = fs_read(&file, pad, 3) == 3;  // reserved header
    success = success && fs_read(&file, &default_id, sizeof(default_id)) == sizeof(default_id);
    success = success && fs_read(&file, &home_id, sizeof(home_id)) == sizeof(home_id);
    success = success && fs_read(&file, &wildcard.flags, sizeof(wildcard.flags)) == sizeof(wildcard.flags);
    success = success && fs_read(&file, &next_id, sizeof(next_id)) == sizeof(next_id);

    if (success) {
        while (num_regions < MAX_REGION_ENTRIES) {
            auto r = &regions[num_regions];

            success = fs_read(&file, &r->id, sizeof(r->id)) == sizeof(r->id);
            success = success && fs_read(&file, &r->parent, sizeof(r->parent)) == sizeof(r->parent);
            success = success && fs_read(&file, r->name, sizeof(r->name)) == sizeof(r->name);
            success = success && fs_read(&file, &r->flags, sizeof(r->flags)) == sizeof(r->flags);
            success = success && fs_read(&file, pad, sizeof(pad)) == sizeof(pad);

            if (!success) break;  // EOF

            if (r->id >= next_id) {   // make sure next_id is valid
                next_id = r->id + 1;
            }
            num_regions++;
        }
    }
    fs_close(&file);
    LOG_INF("Loaded %d regions from %s", num_regions, filepath);
    return true;
}

bool RegionMap::save(const char* path) {
    const char* filepath = path;

    // Remove old file first
    fs_unlink(filepath);

    struct fs_file_t file;
    fs_file_t_init(&file);

    if (fs_open(&file, filepath, FS_O_CREATE | FS_O_WRITE) < 0) {
        LOG_ERR("Failed to open %s for write", filepath);
        return false;
    }

    uint8_t pad[128];
    memset(pad, 0, sizeof(pad));

    bool success = fs_write(&file, pad, 3) == 3;  // reserved header
    success = success && fs_write(&file, &default_id, sizeof(default_id)) == sizeof(default_id);
    success = success && fs_write(&file, &home_id, sizeof(home_id)) == sizeof(home_id);
    success = success && fs_write(&file, &wildcard.flags, sizeof(wildcard.flags)) == sizeof(wildcard.flags);
    success = success && fs_write(&file, &next_id, sizeof(next_id)) == sizeof(next_id);

    if (success) {
        for (int i = 0; i < num_regions; i++) {
            auto r = &regions[i];

            success = fs_write(&file, &r->id, sizeof(r->id)) == sizeof(r->id);
            success = success && fs_write(&file, &r->parent, sizeof(r->parent)) == sizeof(r->parent);
            success = success && fs_write(&file, r->name, sizeof(r->name)) == sizeof(r->name);
            success = success && fs_write(&file, &r->flags, sizeof(r->flags)) == sizeof(r->flags);
            success = success && fs_write(&file, pad, sizeof(pad)) == sizeof(pad);

            if (!success) break;  // write failed
        }
    }
    fs_close(&file);
    LOG_INF("Saved %d regions to %s", num_regions, filepath);
    return true;
}

RegionEntry* RegionMap::putRegion(const char* name, uint16_t parent_id, uint16_t id) {
    const char* sp = name;  // check for illegal name chars
    while (*sp) {
        if (!is_name_char(*sp)) return nullptr;  // error
        sp++;
    }

    auto region = findByName(name);
    if (region) {
        if (region->id == parent_id) return nullptr;  // ERROR: invalid parent!
        region->parent = parent_id;  // re-parent / move this region in the hierarchy
    } else {
        if (id == 0 && num_regions >= MAX_REGION_ENTRIES) return nullptr;  // full!

        region = &regions[num_regions++];  // alloc new RegionEntry
        region->flags = REGION_DENY_FLOOD;  // DENY by default
        region->id = id == 0 ? next_id++ : id;
        StrHelper::strncpy(region->name, name, sizeof(region->name));
        region->parent = parent_id;
    }
    return region;
}

int RegionMap::getTransportKeysFor(const RegionEntry& src, TransportKey dest[], int max_num) {
    int num;
    if (src.name[0] == '$') {  // private region
        num = _store->loadKeysFor(src.id, dest, max_num);
    } else if (src.name[0] == '#') {  // auto hashtag region
        _store->getAutoKeyFor(src.id, src.name, dest[0]);
        num = 1;
    } else {  // new: implicit auto hashtag region
        char tmp[sizeof(src.name) + 1];
        tmp[0] = '#';
        memcpy(&tmp[1], src.name, sizeof(src.name) - 1);
        tmp[sizeof(src.name)] = '\0';
        _store->getAutoKeyFor(src.id, tmp, dest[0]);
        num = 1;
    }
    return num;
}

RegionEntry* RegionMap::findMatch(mesh::Packet* packet, uint8_t mask) {
    for (int i = 0; i < num_regions; i++) {
        auto region = &regions[i];
        if ((region->flags & mask) == 0) {  // does region allow this? (per 'mask' param)
            TransportKey keys[4];
            int num = getTransportKeysFor(*region, keys, 4);
            for (int j = 0; j < num; j++) {
                uint16_t code = keys[j].calcTransportCode(packet);
                if (packet->transport_codes[0] == code) {  // a match!!
                    return region;
                }
            }
        }
    }
    return nullptr;  // no matches
}

RegionEntry* RegionMap::findByName(const char* name) {
    if (strcmp(name, "*") == 0) return &wildcard;

    if (*name == '#') { name++; }  // ignore the '#' when matching by name
    for (int i = 0; i < num_regions; i++) {
        auto region = &regions[i];
        if (strcmp(name, skip_hash(region->name)) == 0) return region;
    }
    return nullptr;  // not found
}

RegionEntry* RegionMap::findByNamePrefix(const char* prefix) {
    if (strcmp(prefix, "*") == 0) return &wildcard;

    if (*prefix == '#') { prefix++; }  // ignore the '#' when matching by name
    RegionEntry* partial = nullptr;
    for (int i = 0; i < num_regions; i++) {
        auto region = &regions[i];
        if (strcmp(prefix, skip_hash(region->name)) == 0) return region;  // complete match
        if (memcmp(prefix, skip_hash(region->name), strlen(prefix)) == 0) {
            partial = region;
        }
    }
    return partial;
}

RegionEntry* RegionMap::findById(uint16_t id) {
    if (id == 0) return &wildcard;  // special root Region

    for (int i = 0; i < num_regions; i++) {
        auto region = &regions[i];
        if (region->id == id) return region;
    }
    return nullptr;  // not found
}

RegionEntry* RegionMap::getHomeRegion() {
    return findById(home_id);
}

void RegionMap::setHomeRegion(const RegionEntry* home) {
    home_id = home ? home->id : 0;
}

RegionEntry* RegionMap::getDefaultRegion() {
    return default_id == 0 ? nullptr : findById(default_id);
}

void RegionMap::setDefaultRegion(const RegionEntry* def) {
    default_id = def ? def->id : 0;
}

bool RegionMap::removeRegion(const RegionEntry& region) {
    if (region.id == 0) return false;  // cannot remove wildcard

    // first check region has no child regions
    for (int i = 0; i < num_regions; i++) {
        if (regions[i].parent == region.id) return false;  // must remove children first
    }

    int i = 0;
    while (i < num_regions) {
        if (region.id == regions[i].id) break;
        i++;
    }
    if (i >= num_regions) return false;  // not found

    num_regions--;  // remove from regions array
    while (i < num_regions) {
        regions[i] = regions[i + 1];
        i++;
    }
    return true;
}

bool RegionMap::clear() {
    num_regions = 0;
    return true;
}

void RegionMap::printChildRegions(int indent, const RegionEntry* parent, char* buf, int& pos, int max_len) const {
    // Print indentation
    for (int i = 0; i < indent && pos < max_len - 1; i++) {
        buf[pos++] = ' ';
    }

    // Print region info
    int written;
    if (parent->flags & REGION_DENY_FLOOD) {
        written = snprintf(&buf[pos], max_len - pos, "%s%s\n",
                          skip_hash(parent->name),
                          parent->id == home_id ? "^" : "");
    } else {
        written = snprintf(&buf[pos], max_len - pos, "%s%s F\n",
                          skip_hash(parent->name),
                          parent->id == home_id ? "^" : "");
    }
    if (written > 0 && pos + written < max_len) {
        pos += written;
    }

    // Print children recursively
    for (int i = 0; i < num_regions; i++) {
        auto r = &regions[i];
        if (r->parent == parent->id) {
            printChildRegions(indent + 1, r, buf, pos, max_len);
        }
    }
}

size_t RegionMap::exportTo(char* dest, size_t max_len) const {
    if (!dest || max_len == 0) return 0;

    int pos = 0;
    printChildRegions(0, &wildcard, dest, pos, (int)max_len);
    return (size_t)pos;
}

int RegionMap::exportNamesTo(char* dest, int max_len, uint8_t mask, bool invert) {
    char* dp = dest;

    // Check wildcard region
    bool wildcard_matches = invert ? (wildcard.flags & mask) : !(wildcard.flags & mask);
    if (wildcard_matches) {
        *dp++ = '*';
        *dp++ = ',';
    }

    for (int i = 0; i < num_regions; i++) {
        auto region = &regions[i];

        // Check if region matches the filter criteria
        bool region_matches = invert ? (region->flags & mask) : !(region->flags & mask);

        if (region_matches) {
            int len = strlen(skip_hash(region->name));
            if ((dp - dest) + len + 2 < max_len) {  // only append if name will fit
                memcpy(dp, skip_hash(region->name), len);
                dp += len;
                *dp++ = ',';
            }
        }
    }

    if (dp > dest) { dp--; }  // don't include trailing comma

    *dp = 0;  // set null terminator
    return dp - dest;
}
