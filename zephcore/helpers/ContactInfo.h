/*
 * SPDX-License-Identifier: MIT
 * ContactInfo - contact/peer info for mesh
 */

#pragma once

#include <mesh/Mesh.h>
#include <mesh/MeshCore.h>

#define OUT_PATH_UNKNOWN   0xFF
/* Marker used in BLE offline-queue frames to indicate the message was sent
 * by THIS device (not received). Also reused by the joystick UI's local
 * message-preview structs for the same purpose. Not a routing value —
 * never appears in over-the-air packet path_len fields. */
#define OUT_PATH_SENT      0xFE

struct ContactInfo {
	mesh::Identity id;
	char name[32];
	uint8_t type;
	uint8_t flags;
	uint8_t out_path_len;
	mutable bool shared_secret_valid;
	uint8_t out_path[MAX_PATH_SIZE];
	uint32_t last_advert_timestamp;
	uint32_t lastmod;
	int32_t gps_lat, gps_lon;
	uint32_t sync_since;

	const uint8_t *getSharedSecret(const mesh::LocalIdentity &self_id) const {
		if (!shared_secret_valid) {
			self_id.calcSharedSecret(shared_secret, id.pub_key);
			shared_secret_valid = true;
		}
		return shared_secret;
	}

private:
	mutable uint8_t shared_secret[PUB_KEY_SIZE];
};
