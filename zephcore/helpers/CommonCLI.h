/*
 * SPDX-License-Identifier: Apache-2.0
 * CommonCLI - Common CLI command handlers for repeaters
 */

#pragma once

#include <zephyr/kernel.h>
#include <mesh/Mesh.h>
#include <mesh/Board.h>
#include <helpers/IdentityStore.h>
#include <helpers/ClientACL.h>
#include "NodePrefs.h"

/* CLI reply buffer size — callers must provide at least this many bytes */
#define CLI_REPLY_SIZE 256

/* Deferred reboot types */
#define REBOOT_NONE       0
#define REBOOT_NORMAL     1
#define REBOOT_DFU        2
#define REBOOT_OTA        3

class CommonCLICallbacks {
public:
    virtual void savePrefs() = 0;
    virtual const char* getFirmwareVer() = 0;
    virtual const char* getBuildDate() = 0;
    virtual const char* getRole() = 0;
    virtual bool formatFileSystem() = 0;
    virtual void sendSelfAdvertisement(int delay_millis, bool flood) = 0;
    virtual void updateAdvertTimer() = 0;
    virtual void updateFloodAdvertTimer() = 0;
    virtual void setLoggingOn(bool enable) = 0;
    virtual void eraseLogFile() = 0;
    virtual void dumpLogFile() = 0;
    virtual void setTxPower(int8_t power_dbm) = 0;
    virtual void formatNeighborsReply(char* reply) = 0;
    virtual void removeNeighbor(const uint8_t* pubkey, int key_len) {
        // no-op by default
    }
    virtual void formatStatsReply(char* reply) = 0;
    virtual void formatRadioStatsReply(char* reply) = 0;
    virtual void formatPacketStatsReply(char* reply) = 0;
    virtual mesh::LocalIdentity& getSelfId() = 0;
    virtual void saveIdentity(const mesh::LocalIdentity& new_id) = 0;
    virtual void clearStats() = 0;
    virtual void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) = 0;

    // Adaptive contention window
    virtual float getContentionEstimate() const { return -1.0f; }
    virtual float getFloodDelayFactor() const { return 0.5f; }
    virtual void setBackoffMultiplier(float m) { (void)m; }

    // Adaptive Power Control
    virtual int8_t getAPCReduction() const { return 0; }
    virtual float getAPCMargin() const { return 0.0f; }
    virtual bool isAPCEnabled() const { return true; }
    virtual void setAPCEnabled(bool en) { (void)en; }
    virtual uint8_t getAPCTargetMargin() const { return 16; }
    virtual void setAPCTargetMargin(uint8_t margin_db) { (void)margin_db; }

    // Sensor manager interface (for GPS)
    virtual double getNodeLat() const { return 0.0; }
    virtual double getNodeLon() const { return 0.0; }
    virtual bool setGpsEnabled(bool enabled) { return false; }
    virtual bool isGpsEnabled() const { return false; }
    virtual int getNumSensorSettings() const { return 0; }
    virtual const char* getSensorSettingName(int idx) const { return nullptr; }
    virtual const char* getSensorSettingValue(int idx) const { return nullptr; }
    virtual const char* getSensorSettingByKey(const char* key) const { return nullptr; }
    virtual bool setSensorSettingValue(const char* key, const char* value) { return false; }
};

class CommonCLI {
    /* Members ordered to match constructor initialization order */
    mesh::MainBoard* _board;
    mesh::RTCClock* _rtc;
    ClientACL* _acl;
    NodePrefs* _prefs;
    CommonCLICallbacks* _callbacks;
    char tmp[PRV_KEY_SIZE * 2 + 4];

    /* Deferred reboot - lets LoRa reply be sent before rebooting */
    struct k_work_delayable _reboot_work;
    uint8_t _pending_reboot;
    static void rebootWorkHandler(struct k_work *work);

    mesh::RTCClock* getRTCClock() { return _rtc; }
    void savePrefs();
    void scheduleReboot(uint8_t type);

public:
    CommonCLI(mesh::MainBoard& board, mesh::RTCClock& rtc, ClientACL& acl,
              NodePrefs* prefs, CommonCLICallbacks* callbacks)
        : _board(&board), _rtc(&rtc), _acl(&acl), _prefs(prefs), _callbacks(callbacks),
          _pending_reboot(REBOOT_NONE)
    {
        k_work_init_delayable(&_reboot_work, rebootWorkHandler);
    }

    void loadPrefs(const char* path);
    void savePrefs(const char* path);
    void handleCommand(uint32_t sender_timestamp, const char* command, char* reply);
    uint8_t buildAdvertData(uint8_t node_type, uint8_t* app_data);
};
