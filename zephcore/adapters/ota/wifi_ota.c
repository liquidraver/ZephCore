/*
 * SPDX-License-Identifier: Apache-2.0
 * WiFi OTA Firmware Update
 *
 * Starts a WiFi AP + HTTP server for browser-based firmware upload.
 * Mirrors Arduino MeshCore's ElegantOTA:
 *   - WiFi SoftAP "MeshCore-OTA" at 192.168.100.1
 *   - DHCP server for client IP assignment
 *   - HTTP server with upload page at /update
 *   - Firmware written to MCUboot slot1 via flash_img API
 *   - Reboot after successful upload (MCUboot activates new image)
 */

#include "wifi_ota.h"
#include "ota_page.h"

#include <zephyr/kernel.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/net/http/server.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(wifi_ota);

/* ========== Configuration ========== */

#define OTA_AP_SSID      CONFIG_ZEPHCORE_OTA_AP_SSID
#define OTA_STATIC_IP    CONFIG_ZEPHCORE_OTA_AP_IP
#define OTA_NETMASK      "255.255.255.0"
#define OTA_DHCP_BASE    "192.168.100.10"

/* ========== State ========== */

static bool ota_active;
static struct flash_img_context flash_ctx;
static size_t total_bytes_received;
static bool flash_ctx_initialized;

/* Identity strings for web page */
static char identity_json[128];
static char home_html[384];

/* ========== Reboot work (delayed to let HTTP response send) ========== */

static void ota_reboot_fn(struct k_work *work)
{
	LOG_INF("OTA reboot");
	sys_reboot(SYS_REBOOT_COLD);
}

static K_WORK_DELAYABLE_DEFINE(ota_reboot_work, ota_reboot_fn);

/* ========== HTTP Service ========== */

static uint16_t http_port = 80;

/* Forward declarations */
static int home_handler(struct http_client_ctx *client,
			enum http_transaction_status status,
			const struct http_request_ctx *request_ctx,
			struct http_response_ctx *response_ctx,
			void *user_data);
static int identity_handler(struct http_client_ctx *client,
			    enum http_transaction_status status,
			    const struct http_request_ctx *request_ctx,
			    struct http_response_ctx *response_ctx,
			    void *user_data);
static int update_page_handler(struct http_client_ctx *client,
			       enum http_transaction_status status,
			       const struct http_request_ctx *request_ctx,
			       struct http_response_ctx *response_ctx,
			       void *user_data);
static int upload_handler(struct http_client_ctx *client,
			  enum http_transaction_status status,
			  const struct http_request_ctx *request_ctx,
			  struct http_response_ctx *response_ctx,
			  void *user_data);

/* HTTP service on port 80 */
HTTP_SERVICE_DEFINE(ota_service, NULL, &http_port, 1, 5, NULL, NULL, NULL);

/* GET / — home page */
static struct http_resource_detail_dynamic home_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
	},
	.cb = home_handler,
};
HTTP_RESOURCE_DEFINE(home_res, ota_service, "/", &home_detail);

/* GET /identity — JSON device info (for JS fetch) */
static struct http_resource_detail_dynamic identity_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
	},
	.cb = identity_handler,
};
HTTP_RESOURCE_DEFINE(identity_res, ota_service, "/identity", &identity_detail);

/* GET /update — upload page (gzip-compressed HTML) */
static struct http_resource_detail_dynamic update_page_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
	},
	.cb = update_page_handler,
};
HTTP_RESOURCE_DEFINE(update_res, ota_service, "/update", &update_page_detail);

/* POST /upload — firmware receive */
static struct http_resource_detail_dynamic upload_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_POST),
	},
	.cb = upload_handler,
};
HTTP_RESOURCE_DEFINE(upload_res, ota_service, "/upload", &upload_detail);

/* ========== HTTP Handlers ========== */

static int home_handler(struct http_client_ctx *client,
			enum http_transaction_status status,
			const struct http_request_ctx *request_ctx,
			struct http_response_ctx *response_ctx,
			void *user_data)
{
	LOG_DBG("home_handler status=%d", status);
	if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
		response_ctx->body = (const uint8_t *)home_html;
		response_ctx->body_len = strlen(home_html);
		response_ctx->final_chunk = true;
		response_ctx->status = HTTP_200_OK;
		LOG_DBG("Served home page (%u bytes)", (unsigned)response_ctx->body_len);
	}
	return 0;
}

static int identity_handler(struct http_client_ctx *client,
			    enum http_transaction_status status,
			    const struct http_request_ctx *request_ctx,
			    struct http_response_ctx *response_ctx,
			    void *user_data)
{
	static const char ct[] = "application/json";

	LOG_DBG("identity_handler status=%d", status);
	if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
		response_ctx->body = (const uint8_t *)identity_json;
		response_ctx->body_len = strlen(identity_json);
		response_ctx->final_chunk = true;
		response_ctx->status = HTTP_200_OK;
		response_ctx->header_count = 1;
		static struct http_header ct_hdr = {
			.name = "Content-Type",
			.value = ct,
		};
		response_ctx->headers = &ct_hdr;
	}
	return 0;
}

static int update_page_handler(struct http_client_ctx *client,
			       enum http_transaction_status status,
			       const struct http_request_ctx *request_ctx,
			       struct http_response_ctx *response_ctx,
			       void *user_data)
{
	LOG_DBG("update_page_handler status=%d", status);
	if (status == HTTP_SERVER_REQUEST_DATA_FINAL) {
		response_ctx->body = ota_page_gz;
		response_ctx->body_len = OTA_PAGE_GZ_SIZE;
		response_ctx->final_chunk = true;
		response_ctx->status = HTTP_200_OK;

		/* Headers for gzip-compressed HTML */
		static struct http_header hdrs[] = {
			{ .name = "Content-Type", .value = "text/html" },
			{ .name = "Content-Encoding", .value = "gzip" },
		};
		response_ctx->headers = hdrs;
		response_ctx->header_count = 2;
	}
	return 0;
}

static int upload_handler(struct http_client_ctx *client,
			  enum http_transaction_status status,
			  const struct http_request_ctx *request_ctx,
			  struct http_response_ctx *response_ctx,
			  void *user_data)
{
	static const char ok_resp[] = "OK";
	static const char fail_resp[] = "FAIL";
	int ret;

	if (status == HTTP_SERVER_REQUEST_DATA_MORE ||
	    status == HTTP_SERVER_REQUEST_DATA_FINAL) {

		/* First chunk: initialize flash context */
		if (!flash_ctx_initialized) {
			ret = flash_img_init(&flash_ctx);
			if (ret) {
				LOG_ERR("flash_img_init failed: %d", ret);
				response_ctx->status = HTTP_500_INTERNAL_SERVER_ERROR;
				response_ctx->body = (const uint8_t *)fail_resp;
				response_ctx->body_len = sizeof(fail_resp) - 1;
				response_ctx->final_chunk = true;
				return 0;
			}
			flash_ctx_initialized = true;
			total_bytes_received = 0;
			LOG_INF("OTA upload started");
		}

		/* Write chunk to flash (slot1) */
		bool is_final = (status == HTTP_SERVER_REQUEST_DATA_FINAL);

		if (request_ctx->data_len > 0) {
			ret = flash_img_buffered_write(&flash_ctx,
						       request_ctx->data,
						       request_ctx->data_len,
						       is_final);
			if (ret) {
				LOG_ERR("Flash write failed at %u bytes: %d",
					(unsigned)total_bytes_received, ret);
				flash_ctx_initialized = false;
				response_ctx->status = HTTP_500_INTERNAL_SERVER_ERROR;
				response_ctx->body = (const uint8_t *)fail_resp;
				response_ctx->body_len = sizeof(fail_resp) - 1;
				response_ctx->final_chunk = true;
				return 0;
			}
			total_bytes_received += request_ctx->data_len;
		}

		if (is_final) {
			LOG_INF("OTA upload complete: %u bytes",
				(unsigned)total_bytes_received);

			/* Mark new image for boot (overwrite-only: permanent) */
			ret = boot_request_upgrade(BOOT_UPGRADE_PERMANENT);
			if (ret) {
				LOG_ERR("boot_request_upgrade failed: %d", ret);
				flash_ctx_initialized = false;
				response_ctx->status = HTTP_500_INTERNAL_SERVER_ERROR;
				response_ctx->body = (const uint8_t *)fail_resp;
				response_ctx->body_len = sizeof(fail_resp) - 1;
				response_ctx->final_chunk = true;
				return 0;
			}

			flash_ctx_initialized = false;

			response_ctx->status = HTTP_200_OK;
			response_ctx->body = (const uint8_t *)ok_resp;
			response_ctx->body_len = sizeof(ok_resp) - 1;
			response_ctx->final_chunk = true;

			/* Reboot after 2s (let HTTP response send) */
			k_work_schedule(&ota_reboot_work, K_SECONDS(2));
		}
	} else if (status == HTTP_SERVER_TRANSACTION_ABORTED) {
		LOG_WRN("OTA upload aborted at %u bytes",
			(unsigned)total_bytes_received);
		flash_ctx_initialized = false;
		total_bytes_received = 0;
	}

	return 0;
}

/* ========== WiFi Event Monitoring ========== */

static struct net_mgmt_event_callback wifi_mgmt_cb;

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				     uint64_t mgmt_event,
				     struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_AP_ENABLE_RESULT:
		LOG_DBG("WiFi AP enable result event received");
		break;
	case NET_EVENT_WIFI_AP_DISABLE_RESULT:
		LOG_DBG("WiFi AP disable result event");
		break;
	case NET_EVENT_WIFI_AP_STA_CONNECTED:
		LOG_INF("WiFi client CONNECTED to AP");
		break;
	case NET_EVENT_WIFI_AP_STA_DISCONNECTED:
		LOG_INF("WiFi client DISCONNECTED from AP");
		break;
	default:
		LOG_DBG("WiFi mgmt event: 0x%016llx", mgmt_event);
		break;
	}
}

/* ========== WiFi AP Setup ========== */

static int wifi_ap_start(void)
{
	struct net_if *iface = net_if_get_default();

	if (!iface) {
		LOG_ERR("No network interface");
		return -ENODEV;
	}

	LOG_DBG("Network iface: %p, idx=%d", iface, net_if_get_by_iface(iface));

	/* Register WiFi management event callback */
	net_mgmt_init_event_callback(&wifi_mgmt_cb, wifi_mgmt_event_handler,
				     NET_EVENT_WIFI_AP_ENABLE_RESULT |
				     NET_EVENT_WIFI_AP_DISABLE_RESULT |
				     NET_EVENT_WIFI_AP_STA_CONNECTED |
				     NET_EVENT_WIFI_AP_STA_DISCONNECTED);
	net_mgmt_add_event_callback(&wifi_mgmt_cb);

	/* Set static IP */
	struct in_addr addr, netmask;

	if (net_addr_pton(AF_INET, OTA_STATIC_IP, &addr)) {
		LOG_ERR("Invalid IP: %s", OTA_STATIC_IP);
		return -EINVAL;
	}
	if (net_addr_pton(AF_INET, OTA_NETMASK, &netmask)) {
		LOG_ERR("Invalid netmask");
		return -EINVAL;
	}

	struct net_if_addr *ifaddr = net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0);
	if (!ifaddr) {
		LOG_ERR("Failed to set static IP %s", OTA_STATIC_IP);
		return -ENOMEM;
	}
	net_if_ipv4_set_netmask_by_addr(iface, &addr, &netmask);
	net_if_ipv4_set_gw(iface, &addr);
	LOG_INF("Static IP set: %s/%s", OTA_STATIC_IP, OTA_NETMASK);

	/* Enable WiFi AP */
	struct wifi_connect_req_params ap_params = {0};

	ap_params.ssid = (const uint8_t *)OTA_AP_SSID;
	ap_params.ssid_length = strlen(OTA_AP_SSID);
	ap_params.channel = WIFI_CHANNEL_ANY;
	ap_params.security = WIFI_SECURITY_TYPE_NONE;
	ap_params.band = WIFI_FREQ_BAND_2_4_GHZ;

	LOG_INF("Enabling WiFi AP: SSID=%s, channel=any, security=open", OTA_AP_SSID);
	int ret = net_mgmt(NET_REQUEST_WIFI_AP_ENABLE, iface,
			   &ap_params, sizeof(ap_params));
	if (ret) {
		LOG_ERR("WiFi AP enable failed: %d", ret);
		return ret;
	}

	LOG_INF("WiFi AP started: SSID=%s", OTA_AP_SSID);

	/* Bring the interface up explicitly */
	if (!net_if_is_up(iface)) {
		LOG_WRN("Interface not up, bringing up...");
		ret = net_if_up(iface);
		if (ret) {
			LOG_ERR("net_if_up failed: %d", ret);
		}
	}

	/* Start DHCP server */
	struct in_addr dhcp_base;

	if (net_addr_pton(AF_INET, OTA_DHCP_BASE, &dhcp_base) == 0) {
		ret = net_dhcpv4_server_start(iface, &dhcp_base);
		if (ret) {
			LOG_WRN("DHCP server start failed: %d (non-fatal)", ret);
		} else {
			LOG_INF("DHCP server started (pool: %s+)", OTA_DHCP_BASE);
		}
	}

	return 0;
}

static int wifi_ap_stop(void)
{
	struct net_if *iface = net_if_get_default();

	if (!iface) {
		return -ENODEV;
	}

	net_dhcpv4_server_stop(iface);

	int ret = net_mgmt(NET_REQUEST_WIFI_AP_DISABLE, iface, NULL, 0);

	if (ret) {
		LOG_WRN("WiFi AP disable failed: %d", ret);
	}

	/* Remove static IP */
	struct in_addr addr;

	if (net_addr_pton(AF_INET, OTA_STATIC_IP, &addr) == 0) {
		net_if_ipv4_addr_rm(iface, &addr);
	}

	return ret;
}

/* ========== Public API ========== */

int wifi_ota_start(const char *node_name, const char *board_name)
{
	if (ota_active) {
		return -EALREADY;
	}

	/* Prepare identity strings for web page */
	snprintf(identity_json, sizeof(identity_json),
		 "{\"name\":\"%s\",\"board\":\"%s\"}",
		 node_name ? node_name : "Unknown",
		 board_name ? board_name : "Unknown");

	snprintf(home_html, sizeof(home_html),
		 "<html><body style=\"background:#1a1a2e;color:#eee;font-family:sans-serif;"
		 "display:flex;justify-content:center;align-items:center;height:100vh\">"
		 "<h2 style=\"color:#0ff\">MeshCore OTA: %s (%s)<br>"
		 "<a href=\"/update\" style=\"color:#00b894\">Go to Update Page</a></h2>"
		 "</body></html>",
		 node_name ? node_name : "Unknown",
		 board_name ? board_name : "Unknown");

	/* Start WiFi AP */
	int ret = wifi_ap_start();

	if (ret) {
		return ret;
	}

	/* Start HTTP server */
	LOG_INF("Starting HTTP server on port %u...", http_port);
	ret = http_server_start();
	if (ret) {
		LOG_ERR("HTTP server start failed: %d", ret);
		wifi_ap_stop();
		return ret;
	}

	ota_active = true;
	flash_ctx_initialized = false;
	total_bytes_received = 0;

	LOG_INF("OTA server ready at http://%s/update", OTA_STATIC_IP);
	LOG_INF("HTTP routes: / (home), /identity (json), /update (upload page), /upload (POST)");
	return 0;
}

int wifi_ota_stop(void)
{
	if (!ota_active) {
		return 0;
	}

	http_server_stop();
	wifi_ap_stop();
	ota_active = false;
	flash_ctx_initialized = false;
	total_bytes_received = 0;

	LOG_INF("OTA server stopped");
	return 0;
}

bool wifi_ota_is_active(void)
{
	return ota_active;
}

void wifi_ota_confirm_image(void)
{
#if IS_ENABLED(CONFIG_BOOTLOADER_MCUBOOT)
	if (!boot_is_img_confirmed()) {
		int ret = boot_write_img_confirmed();

		if (ret) {
			LOG_ERR("Failed to confirm MCUboot image: %d", ret);
		} else {
			LOG_INF("MCUboot image confirmed");
		}
	}
#endif
}
