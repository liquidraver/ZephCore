/*
 * SPDX-License-Identifier: MIT
 * Companion protocol: opcodes, response and error codes (upstream keeps these
 * at the top of examples/companion_radio/MyMesh.cpp).
 */

#pragma once

#include <stdint.h>

/* Protocol commands (matches Arduino companion_radio) - sorted by opcode */
#define CMD_APP_START            0x01
#define CMD_SEND_TXT_MSG         0x02
#define CMD_SEND_CHANNEL_TXT_MSG 0x03
#define CMD_GET_CONTACTS         0x04
#define CMD_GET_DEVICE_TIME      0x05
#define CMD_SET_DEVICE_TIME      0x06
#define CMD_SEND_SELF_ADVERT     0x07
#define CMD_SET_ADVERT_NAME      0x08
#define CMD_ADD_UPDATE_CONTACT   0x09
#define CMD_SYNC_NEXT_MESSAGE    0x0A
#define CMD_SET_RADIO_PARAMS     0x0B
#define CMD_SET_RADIO_TX_POWER   0x0C
#define CMD_RESET_PATH           0x0D
#define CMD_SET_ADVERT_LATLON    0x0E
#define CMD_REMOVE_CONTACT       0x0F
#define CMD_SHARE_CONTACT        0x10
#define CMD_EXPORT_CONTACT       0x11
#define CMD_IMPORT_CONTACT       0x12
#define CMD_REBOOT               0x13
#define CMD_GET_BATT_AND_STORAGE 0x14
#define CMD_SET_TUNING_PARAMS    0x15
#define CMD_DEVICE_QUERY         0x16
#define CMD_EXPORT_PRIVATE_KEY   0x17
#define CMD_IMPORT_PRIVATE_KEY   0x18
#define CMD_SEND_RAW_DATA        0x19
#define CMD_SEND_LOGIN           0x1A
#define CMD_SEND_STATUS_REQ      0x1B
#define CMD_HAS_CONNECTION       0x1C
#define CMD_LOGOUT               0x1D
#define CMD_GET_CONTACT_BY_KEY   0x1E
#define CMD_GET_CHANNEL          0x1F
#define CMD_SET_CHANNEL          0x20
#define CMD_SIGN_START           0x21
#define CMD_SIGN_DATA            0x22
#define CMD_SIGN_FINISH          0x23
#define CMD_SEND_TRACE_PATH      0x24
#define CMD_SET_DEVICE_PIN       0x25
#define CMD_SET_OTHER_PARAMS     0x26
#define CMD_SEND_TELEMETRY_REQ   0x27
#define CMD_GET_CUSTOM_VARS      0x28
#define CMD_SET_CUSTOM_VAR       0x29
#define CMD_GET_ADVERT_PATH      0x2A
#define CMD_GET_TUNING_PARAMS    0x2B
#define CMD_SEND_BINARY_REQ      0x32
#define CMD_FACTORY_RESET        0x33
#define CMD_SEND_PATH_DISCOVERY_REQ  0x34
#define CMD_SET_FLOOD_SCOPE_KEY  0x36  /* v8+ (renamed from CMD_SET_FLOOD_SCOPE) */
#define CMD_SEND_CONTROL_DATA    0x37
#define CMD_GET_STATS            0x38
#define CMD_SEND_ANON_REQ        0x39
#define CMD_SET_AUTOADD_CONFIG   0x3A
#define CMD_GET_AUTOADD_CONFIG   0x3B
#define CMD_GET_ALLOWED_REPEAT_FREQ 0x3C
#define CMD_SET_PATH_HASH_MODE      0x3D
#define CMD_SEND_CHANNEL_DATA       0x3E
#define CMD_SET_DEFAULT_FLOOD_SCOPE 0x3F  /* v11+ */
#define CMD_GET_DEFAULT_FLOOD_SCOPE 0x40  /* v11+ */
#define CMD_SEND_RAW_PACKET         0x41  /* v12+ */
#define CMD_RUN_CLI_COMMAND         0x42  /* v14+ */

/* Response packet types */
#define PACKET_OK               0x00
#define PACKET_ERROR            0x01
#define PACKET_CONTACT_START    0x02
#define PACKET_CONTACT          0x03
#define PACKET_CONTACT_END      0x04
#define PACKET_SELF_INFO        0x05
#define PACKET_SENT             0x06
#define PACKET_CONTACT_MSG_RECV 0x07  /* Legacy - ver < 3 */
#define PACKET_CHANNEL_MSG_RECV 0x08  /* Legacy - ver < 3 */
#define PACKET_CURR_TIME        0x09
#define PACKET_NO_MORE_MSGS     0x0A
#define PACKET_EXPORT_CONTACT   0x0B
#define PACKET_BATTERY          0x0C
#define PACKET_DEVICE_INFO      0x0D
#define PACKET_PRIVATE_KEY      0x0E
#define PACKET_DISABLED         0x0F
#define PACKET_CONTACT_MSG_V3   0x10  /* Contact message for app ver >= 3 */
#define PACKET_CHANNEL_MSG_V3   0x11  /* Channel message for app ver >= 3 */
#define PACKET_CHANNEL_INFO     0x12
#define PACKET_SIGN_START       0x13
#define PACKET_SIGNATURE        0x14
#define PACKET_CUSTOM_VARS      0x15
#define PACKET_ADVERT_PATH      0x16
#define PACKET_TUNING_PARAMS    0x17
#define PACKET_STATS            0x18
#define PACKET_AUTOADD_CONFIG   0x19
#define PACKET_ALLOWED_REPEAT_FREQ 0x1A
#define PACKET_CHANNEL_DATA_RECV   0x1B
#define PACKET_DEFAULT_FLOOD_SCOPE 0x1C
#define PACKET_CLI_REPLY           0x1D  /* v14+, reply to CMD_RUN_CLI_COMMAND */

#define MAX_CHANNEL_DATA_LENGTH    (MAX_FRAME_SIZE - 9)

#define MAX_SIGN_DATA_LEN       (8 * 1024)

/* Error codes */
#define ERR_UNSUPPORTED         0x01
#define ERR_NOT_FOUND           0x02
#define ERR_TABLE_FULL          0x03
#define ERR_BAD_STATE           0x04
#define ERR_ILLEGAL_ARG         0x06

/* Max LoRa TX power (dBm) for SX1262 */
#define MAX_LORA_TX_POWER       22

/* Helper to put little-endian values */
static inline void put_le16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static inline void put_le32(uint8_t *p, uint32_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF; }
static inline uint32_t get_le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24); }
