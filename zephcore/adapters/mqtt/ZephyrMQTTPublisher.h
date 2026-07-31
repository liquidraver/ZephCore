/*
 * SPDX-License-Identifier: MIT
 * ZephyrMQTTPublisher — MQTT client thread for the Observer role.
 *
 * Dedicated thread: waits for WiFi ready, resolves broker hostname,
 * connects with optional TLS (TLS_PEER_VERIFY_NONE), publishes LWT and
 * status, then drains a pre-serialized JSON publish queue.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ObserverCreds;

/* The publisher only ever writes to the two topics registered in
 * mqtt_publisher_start(); messages carry a 1-byte index instead of a
 * per-message topic string. */
enum mqtt_pub_topic {
	MQTT_PUB_TOPIC_STATUS  = 0,
	MQTT_PUB_TOPIC_PACKETS = 1,
};

/*
 * Start the MQTT publisher thread.
 *   creds         — runtime credentials; pointer kept, must remain valid.
 *   client_id     — MQTT client identifier string (e.g. "Observer-AABBCCDD").
 *   status_topic  — topic for retained online/offline status messages.
 *   packets_topic — topic for received LoRa packet JSON messages.
 *
 * Must be called once from main() after wifi_station_start().
 */
void mqtt_publisher_start(const struct ObserverCreds *creds,
			  const char *client_id,
			  const char *status_topic,
			  const char *packets_topic);

/*
 * Zero-copy publish staging (single producer — the mesh main thread ONLY).
 * mqtt_publisher_stage() returns the staging payload buffer (capacity in
 * *size); the caller builds the JSON directly in it, then calls
 * mqtt_publisher_commit() which copies the staged message into the queue.
 * Drops the message (with a warning) if the queue is full.
 * NOT safe to call from any other thread — the staging slot is unlocked.
 */
char *mqtt_publisher_stage(size_t *size);
void mqtt_publisher_commit(enum mqtt_pub_topic topic, int payload_len);

/* Returns true when the MQTT session is active. */
bool mqtt_publisher_is_connected(void);

/*
 * Trigger an MQTT reconnect (e.g. after credential change via CLI).
 * Safe to call from any thread.
 */
void mqtt_publisher_reconnect(void);

/*
 * Register a callback invoked once each time MQTT connects (CONNACK received).
 * Called from the MQTT publisher thread — keep it short.
 * Pass NULL to clear.
 */
void mqtt_publisher_set_connect_cb(void (*cb)(void));

#ifdef __cplusplus
}
#endif
