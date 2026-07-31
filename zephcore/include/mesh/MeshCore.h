/*
 * SPDX-License-Identifier: MIT
 * ZephCore constants and base types
 */

#pragma once

#include <stdint.h>
#include <math.h>

/* Neutralize STM32 CMSIS peripheral-instance macros (RNG, AES, ...) that
 * collide with portable mesh identifiers. No-op off STM32. */
#include <mesh/stm32_cmsis_fixup.h>

#define MAX_HASH_SIZE        8   /* SHA256 truncated to 8 bytes for dedup */
#define PUB_KEY_SIZE        32   /* Ed25519 public key */
#define PRV_KEY_SIZE        64   /* Ed25519 expanded private key */
#define SEED_SIZE           32   /* Ed25519 seed */
#define SIGNATURE_SIZE      64   /* Ed25519 signature */
#define MAX_ADVERT_DATA_SIZE  32
#define CIPHER_KEY_SIZE     16   /* AES-128 */
#define CIPHER_BLOCK_SIZE   16   /* AES block size */
#define CIPHER_MAC_SIZE      2   /* truncated MAC for bandwidth savings */
#define PATH_HASH_SIZE       1   /* 1-byte per-hop hash in path field */

#define MAX_PACKET_PAYLOAD  184  /* fits in SX126x 255-byte FIFO with headers */
#define MAX_PATH_SIZE        64  /* max hops * max hash size */
#define MAX_TRANS_UNIT      255  /* SX126x FIFO limit */
#define MAX_GROUP_DATA_LENGTH (MAX_PACKET_PAYLOAD - CIPHER_BLOCK_SIZE - 3)

#define MESH_DEBUG_PRINT(...)
#define MESH_DEBUG_PRINTLN(...)
