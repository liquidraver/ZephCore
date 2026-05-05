/*
 * SPDX-License-Identifier: Apache-2.0
 * Zephyr DataStore - LittleFS-backed persistence with optional QSPI flash
 *
 * All platforms use DTS-automounted /lfs for identity, prefs, contacts.
 * QSPI /ext overrides contacts path when available (any platform).
 */

#pragma once

#include <mesh/Identity.h>
#include <mesh/RTC.h>
#include <NodePrefs.h>
#include <ContactInfo.h>
#include <ChannelDetails.h>

class DataStoreHost {
public:
	virtual bool onContactLoaded(const ContactInfo &contact) = 0;
	virtual bool getContactForSave(uint32_t idx, ContactInfo &contact) = 0;
	virtual bool onChannelLoaded(uint8_t channel_idx, const ChannelDetails &ch) = 0;
	virtual bool getChannelForSave(uint8_t channel_idx, ChannelDetails &ch) = 0;
};

class ZephyrDataStore {
public:
	explicit ZephyrDataStore(mesh::RTCClock &clock);
	void begin();
	bool formatFileSystem();
	bool loadMainIdentity(mesh::LocalIdentity &identity);
	bool saveMainIdentity(const mesh::LocalIdentity &identity);
	void loadPrefs(NodePrefs &prefs);
	void savePrefs(const NodePrefs &prefs);
	void loadContacts(DataStoreHost *host);
	void saveContacts(DataStoreHost *host);
	void loadChannels(DataStoreHost *host);
	void saveChannels(DataStoreHost *host);
	uint8_t getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[]);
	bool putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], uint8_t len);
	bool deleteBlobByKey(const uint8_t key[], int key_len);
	/* Used/total KiB for BLE storage report: /ext when mounted, else /lfs (Arduino parity). */
	uint32_t getStorageUsedKb() const;
	uint32_t getStorageTotalKb() const;

	/* Factory reset - delete all stored data */
	void factoryReset();

	/* Check if external QSPI flash is available */
	bool hasExternalStorage() const { return _has_ext_fs; }
	uint32_t getExternalStorageKb() const;

	static bool mount();
	static void unmount();
	static const char *mountPoint() { return MNT_POINT; }
	static const char *extMountPoint() { return EXT_MNT_POINT; }

private:
	/* Internal flash (always available) - identity, prefs */
	static constexpr const char *MNT_POINT = "/lfs";
	static constexpr const char *PREFS_FILE = "/lfs/new_prefs";
	static constexpr const char *MAIN_ID_FILE = "/lfs/_main.id";

	/* External QSPI flash (optional) - contacts, channels, blobs */
	static constexpr const char *EXT_MNT_POINT = "/ext";
	static constexpr const char *EXT_CONTACTS_FILE = "/ext/contacts3";
	static constexpr const char *EXT_CHANNELS_FILE = "/ext/channels2";
	static constexpr const char *EXT_ADV_BLOBS_FILE = "/ext/adv_blobs";

	/* Fallback to internal if no external */
	static constexpr const char *INT_CONTACTS_FILE = "/lfs/contacts3";
	static constexpr const char *INT_CHANNELS_FILE = "/lfs/channels2";
	static constexpr const char *INT_ADV_BLOBS_FILE = "/lfs/adv_blobs";

	mesh::RTCClock *_clock;
	bool _has_ext_fs;

	/* Get path based on external availability */
	const char *contactsFile() const { return _has_ext_fs ? EXT_CONTACTS_FILE : INT_CONTACTS_FILE; }
	const char *channelsFile() const { return _has_ext_fs ? EXT_CHANNELS_FILE : INT_CHANNELS_FILE; }
	const char *advBlobsFile() const { return _has_ext_fs ? EXT_ADV_BLOBS_FILE : INT_ADV_BLOBS_FILE; }
	int maxBlobRecs() const { return _has_ext_fs ? 100 : 20; }

	void checkAdvBlobFile();
	void migrateToExternalFS();
	bool openRead(const char *path, uint8_t *buf, size_t buf_sz, size_t &out_len);
	bool atomicReplaceFile(const char *path, const uint8_t *buf, size_t len);
	bool exists(const char *path);
	bool removeFile(const char *path);
	bool copyFile(const char *src, const char *dst);
};
