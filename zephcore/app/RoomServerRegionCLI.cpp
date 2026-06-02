/*
 * SPDX-License-Identifier: Apache-2.0
 * RoomServerMesh region-definition CLI — the `region ...` command family and the
 * `region load` continuation-line parser.
 *
 * Split out of RoomServerMesh.cpp for readability. Defines two RoomServerMesh
 * methods invoked from RoomServerMesh::handleCommand(); the file-static parser
 * helpers below are used only by this command family.
 */

#include "RoomServerMesh.h"
#include <mesh/Utils.h>
#include <helpers/TxtDataHelpers.h>

#include <stdio.h>
#include <string.h>

/* ---------- region def helpers ---------- */
static char* skipSpaces(char* s) { while (*s == ' ') s++; return s; }
static void rtrimSpaces(char* s) { char* e = s + strlen(s); while (e > s && e[-1] == ' ') *--e = '\0'; }
static char* takeToken(char** cursor) {
    char* p = skipSpaces(*cursor);
    if (*p == '\0') { *cursor = p; return nullptr; }
    char* tok = p;
    while (*p && *p != ' ') p++;
    if (*p) *p++ = '\0';
    *cursor = p;
    return tok;
}
static char* splitNameJump(char* tok) {
    for (char* q = tok; *q; q++) {
        if (*q == '|' || *q == ',') {
            *q = '\0';
            char* jump = skipSpaces(q + 1);
            rtrimSpaces(jump);
            return jump;
        }
    }
    return nullptr;
}
static bool processRegionDefSegment(RegionMap* map, char* tok, RegionEntry** cursor, char* reply) {
    char* jump = splitNameJump(tok);
    char* name = skipSpaces(tok);
    if (*name == '\0') { snprintf(reply, 160, "Err - empty name"); return false; }
    if (jump && *jump == '\0') { snprintf(reply, 160, "Err - empty jump"); return false; }
    RegionEntry* r = map->putRegion(name, (*cursor)->id);
    if (r == NULL) { snprintf(reply, 160, "Err - put failed: %s", name); return false; }
    r->flags = 0;
    if (jump) {
        RegionEntry* j = map->findByNamePrefix(jump);
        if (j == NULL) { snprintf(reply, 160, "Err - unknown jump: %s", jump); return false; }
        *cursor = j;
    } else {
        *cursor = r;
    }
    return true;
}
/* ---------------------------------------- */

void RoomServerMesh::handleRegionLoadLine(char* command, char* reply) {
    if (StrHelper::isBlank(command)) {
        region_map = temp_map;
        region_load_active = false;
        sprintf(reply, "OK - loaded %d regions", region_map.getCount());
    } else {
        char* np = command;
        while (*np == ' ') np++;
        int indent = np - command;

        char* ep = np;
        while (RegionMap::is_name_char(*ep)) ep++;
        if (*ep) { *ep++ = 0; }

        while (*ep && *ep != 'F') ep++;

        if (indent > 0 && indent < 8 && strlen(np) > 0) {
            auto parent = load_stack[indent - 1];
            if (parent) {
                auto old = region_map.findByName(np);
                auto nw = temp_map.putRegion(np, parent->id, old ? old->id : 0);
                if (nw) {
                    nw->flags = old ? old->flags : (*ep == 'F' ? 0 : REGION_DENY_FLOOD);
                    load_stack[indent] = nw;
                }
            }
        }
        reply[0] = 0;
    }
}

void RoomServerMesh::handleRegionCommand(char* command, char* reply) {
    reply[0] = 0;

    // `region def`: cursor-walk bulk region builder — must run before parseTextParts
    // mutates and truncates the buffer to 4 segments.
    char* cmd = skipSpaces(command);
    if (strncmp(cmd, "region def", 10) == 0 && (cmd[10] == ' ' || cmd[10] == '\0')) {
        char* payload = skipSpaces(cmd + 10);
        rtrimSpaces(payload);
        if (*payload == '\0') { snprintf(reply, 160, "Err - empty def"); goto region_done; }
        RegionEntry* cursor = &region_map.getWildcard();
        for (char* tok; (tok = takeToken(&payload)) != nullptr; ) {
            if (!processRegionDefSegment(&region_map, tok, &cursor, reply)) goto region_done;
        }
        region_map.exportTo(reply, 160);
        goto region_done;
    }

    {
    const char* parts[4];
    int n = mesh::Utils::parseTextParts(command, parts, 4, ' ');

    if (n == 1) {
        region_map.exportTo(reply, 160);
    } else if (n >= 2 && strcmp(parts[1], "load") == 0) {
        temp_map.resetFrom(region_map);
        memset(load_stack, 0, sizeof(load_stack));
        load_stack[0] = &temp_map.getWildcard();
        region_load_active = true;
    } else if (n >= 2 && strcmp(parts[1], "save") == 0) {
        _prefs.discovery_mod_timestamp = getRTCClock()->getCurrentTime();
        savePrefs();
        bool success = region_map.save(_store->getRegionsPath());
        strcpy(reply, success ? "OK" : "Err - save failed");
    } else if (n >= 3 && strcmp(parts[1], "allowf") == 0) {
        auto region = region_map.findByNamePrefix(parts[2]);
        if (region) {
            region->flags &= ~REGION_DENY_FLOOD;
            strcpy(reply, "OK");
        } else {
            strcpy(reply, "Err - unknown region");
        }
    } else if (n >= 3 && strcmp(parts[1], "denyf") == 0) {
        auto region = region_map.findByNamePrefix(parts[2]);
        if (region) {
            region->flags |= REGION_DENY_FLOOD;
            strcpy(reply, "OK");
        } else {
            strcpy(reply, "Err - unknown region");
        }
    } else if (n >= 3 && strcmp(parts[1], "get") == 0) {
        auto region = region_map.findByNamePrefix(parts[2]);
        if (region) {
            auto parent = region_map.findById(region->parent);
            if (parent && parent->id != 0) {
                sprintf(reply, " %s (%s) %s", region->name, parent->name, (region->flags & REGION_DENY_FLOOD) ? "" : "F");
            } else {
                sprintf(reply, " %s %s", region->name, (region->flags & REGION_DENY_FLOOD) ? "" : "F");
            }
        } else {
            strcpy(reply, "Err - unknown region");
        }
    } else if (n >= 3 && strcmp(parts[1], "home") == 0) {
        auto home = region_map.findByNamePrefix(parts[2]);
        if (home) {
            region_map.setHomeRegion(home);
            sprintf(reply, " home is now %s", home->name);
        } else {
            strcpy(reply, "Err - unknown region");
        }
    } else if (n == 2 && strcmp(parts[1], "home") == 0) {
        auto home = region_map.getHomeRegion();
        sprintf(reply, " home is %s", home ? home->name : "*");
    } else if (n >= 3 && strcmp(parts[1], "default") == 0) {
        if (strcmp(parts[2], "<null>") == 0) {
            region_map.setDefaultRegion(nullptr);
            memset(default_scope.key, 0, sizeof(default_scope.key));
            region_map.save(_store->getRegionsPath());  // persist in one atomic step
            sprintf(reply, " default scope is now <null>");
        } else {
            auto def = region_map.findByNamePrefix(parts[2]);
            if (def == nullptr) {
                def = region_map.putRegion(parts[2], 0);  // auto-create the default region
            }
            if (def) {
                def->flags = 0;   // make sure allow flood enabled
                region_map.setDefaultRegion(def);
                region_map.getTransportKeysFor(*def, &default_scope, 1);
                region_map.save(_store->getRegionsPath());  // persist in one atomic step
                sprintf(reply, " default scope is now %s", def->name);
            } else {
                strcpy(reply, "Err - region table full");
            }
        }
    } else if (n == 2 && strcmp(parts[1], "default") == 0) {
        auto def = region_map.getDefaultRegion();
        sprintf(reply, " default scope is %s", def ? def->name : "<null>");
    } else if (n >= 3 && strcmp(parts[1], "put") == 0) {
        auto parent = n >= 4 ? region_map.findByNamePrefix(parts[3]) : &region_map.getWildcard();
        if (parent == nullptr) {
            strcpy(reply, "Err - unknown parent");
        } else {
            auto region = region_map.putRegion(parts[2], parent->id);
            if (region == nullptr) {
                strcpy(reply, "Err - unable to put");
            } else {
                region->flags = 0;   // New default: enable flood
                strcpy(reply, "OK - (flood allowed)");
            }
        }
    } else if (n >= 3 && strcmp(parts[1], "remove") == 0) {
        auto region = region_map.findByName(parts[2]);
        if (region) {
            if (region_map.removeRegion(*region)) {
                strcpy(reply, "OK");
            } else {
                strcpy(reply, "Err - not empty");
            }
        } else {
            strcpy(reply, "Err - not found");
        }
    } else if (n >= 3 && strcmp(parts[1], "list") == 0) {
        uint8_t mask = 0;
        bool invert = false;
        if (strcmp(parts[2], "allowed") == 0) {
            mask = REGION_DENY_FLOOD;
            invert = false;
        } else if (strcmp(parts[2], "denied") == 0) {
            mask = REGION_DENY_FLOOD;
            invert = true;
        } else {
            strcpy(reply, "Err - use 'allowed' or 'denied'");
            return;
        }
        int len = region_map.exportNamesTo(reply, 160, mask, invert);
        if (len == 0) {
            strcpy(reply, "-none-");
        }
    } else {
        strcpy(reply, "Err - ??");
    }
    } // end parseTextParts scope
    region_done:;
}
