/*
 * SPDX-License-Identifier: MIT
 * WiFi companion — see CompanionWifi.h. The CLI below is upstream's
 * (examples/companion_radio/MyMesh.cpp, ENABLE_WIFI_INTERFACE), on our prefs
 * and ZephyrWiFiStation instead of Arduino's WiFi class.
 */

#include "CompanionWifi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <helpers/TxtDataHelpers.h>
#include <ZephyrWiFiStation.h>

LOG_MODULE_REGISTER(zephcore_wifi_companion, CONFIG_ZEPHCORE_MAIN_LOG_LEVEL);

/* The station reads its SSID and password at every (re)connect; these copies
 * keep a CLI change from taking effect before the reboot it asks for. */
static char s_ssid[sizeof(((NodePrefs *)0)->wifi_ssid)];
static char s_pwd[sizeof(((NodePrefs *)0)->wifi_pwd)];

void companion_wifi_start(const NodePrefs &prefs)
{
	if (!prefs.wifi_enabled) {
		LOG_INF("WiFi companion off (wifi.enabled 0)");
		return;
	}
	if (prefs.wifi_ssid[0] == '\0') {
		LOG_INF("WiFi companion: no network set (set wifi.ssid, wifi.pwd, reboot)");
		return;
	}
	memcpy(s_ssid, prefs.wifi_ssid, sizeof(s_ssid));
	memcpy(s_pwd, prefs.wifi_pwd, sizeof(s_pwd));
	LOG_INF("WiFi companion: joining %s", s_ssid);
	/* No SNTP time: the app sets the companion's clock, and time only moves
	 * forward (see time_sync.h). */
	zc_wifi_station_start(s_ssid, s_pwd, nullptr);
}

bool companion_wifi_cli(const char *command, NodePrefs &prefs, void (*save)(void), char *reply)
{
	if (memcmp(command, "set wifi.ssid ", 14) == 0) {
		StrHelper::strncpy(prefs.wifi_ssid, &command[14], sizeof(prefs.wifi_ssid));
		save();
		sprintf(reply, "> wifi.ssid is now %s (set wifi.pwd too, then reboot)", prefs.wifi_ssid);
		return true;
	}
	if (memcmp(command, "set wifi.pwd ", 13) == 0) {
		StrHelper::strncpy(prefs.wifi_pwd, &command[13], sizeof(prefs.wifi_pwd));
		save();
		strcpy(reply, "> wifi.pwd updated (reboot to apply)");
		return true;
	}
	if (strcmp(command, "get wifi.pwd") == 0) {
		sprintf(reply, "> %s", prefs.wifi_pwd);
		return true;
	}
	if (strcmp(command, "set wifi.clear") == 0) {
		prefs.wifi_ssid[0] = 0;
		prefs.wifi_pwd[0] = 0;
		save();
		strcpy(reply, "> wifi config cleared (reboot to apply)");
		return true;
	}
	if (strcmp(command, "get wifi.ssid") == 0) {
		sprintf(reply, "> %s", prefs.wifi_ssid[0] ? prefs.wifi_ssid : "(not set)");
		return true;
	}
	if (memcmp(command, "set wifi.enabled ", 17) == 0) {
		prefs.wifi_enabled = atoi(&command[17]) ? 1 : 0;
		save();
		sprintf(reply, "> wifi.enabled is now %d (reboot to apply)", prefs.wifi_enabled);
		return true;
	}
	if (strcmp(command, "get wifi.enabled") == 0) {
		sprintf(reply, "> %d", prefs.wifi_enabled);
		return true;
	}
	if (strcmp(command, "get wifi.status") == 0) {
		strcpy(reply, zc_wifi_station_is_connected() ? "> connected" : "> disconnected");
		return true;
	}
	if (strcmp(command, "get wifi.ip") == 0) {
		char ip[16];

		if (zc_wifi_station_ip(ip, sizeof(ip))) {
			sprintf(reply, "> %s", ip);
		} else {
			strcpy(reply, "> (not connected)");
		}
		return true;
	}
	return false;
}
