/*
 * SPDX-License-Identifier: MIT
 * PrefsJson - prefs.json for both roles. See PrefsJson.h.
 */

#include "PrefsJson.h"

namespace {

/* radio{} and gps{}: the same keys in both of upstream's trees. Not written:
 * "cad" (ZephCore always runs CAD), "agc_int" (periodic AGC reset removed),
 * "fem_txgain" (no such setting here). */
class RadioJson : public ConfigSerializer {
	NodePrefs *_p;

protected:
	void structure() override
	{
		def("freq", _p->freq);
		def("bw", _p->bw);
		def("sf", _p->sf);
		def("cr", _p->cr);
		def("int_thr", _p->interference_threshold);
		def("rxgain", _p->rx_boost);
		def("fem_rxgain", _p->fem_rxgain);
		def("tx", _p->tx_power_dbm);
		def("af", _p->airtime_factor);
		def("rxdelay", _p->rx_delay_base);
		def("f_txdelay", _p->tx_delay_factor);
		def("d_txdelay", _p->direct_tx_delay_factor);
		def("hash_mode", _p->path_hash_mode);
		def("multi_ack", _p->multi_acks);
	}

public:
	explicit RadioJson(NodePrefs *p) : _p(p) {}
};

class GpsJson : public ConfigSerializer {
	NodePrefs *_p;

protected:
	void structure() override
	{
		def("en", _p->gps_enabled);
		def("int", _p->gps_interval);
		def("adv_loc", _p->advert_loc_policy);
	}

public:
	explicit GpsJson(NodePrefs *p) : _p(p) {}
};

/* zc{}: ZephCore's own settings, common to both roles */
class ZcCommonJson : public ConfigSerializer {
protected:
	NodePrefs *_p;

	void structure() override
	{
		def("leds_off", _p->leds_disabled);
		def("leds_radio", _p->leds_radio_mode);
		def("leds_hb", _p->leds_hb_mode);
		def("rx_dc", _p->rx_duty_cycle);
		def("mts", _p->meshtimesync);
		def("cad_auto", _p->cad_auto);
		def("cad_off", _p->cad_offset);
		def("cad_base", _p->cad_base);
		def("probe_int", _p->probe_interval);
		def("busycap", _p->cad_busycap);
		def("xsf", (void *)_p->extra_sf, EXTRA_SF_MAX);
		def("disp_rot", _p->display_rotate);
		def("in_rot", _p->input_rotate);
	}

public:
	explicit ZcCommonJson(NodePrefs *p) : _p(p) {}
};

/* ── Companion: upstream examples/companion_radio/NodePrefs.h ──────── */

class RepeatDisableJson : public ConfigSerializer {
protected:
	void structure() override { def("disable", disable); }

public:
	uint8_t disable = 1;
};

class CompJson : public ConfigSerializer {
	NodePrefs *_p;

protected:
	void structure() override
	{
		def("auto_max", _p->autoadd_max_hops);
		def("defs_nm", _p->default_scope_name, sizeof(_p->default_scope_name));
		def("defs_key", (void *)_p->default_scope_key, sizeof(_p->default_scope_key));
		def("pin", _p->ble_pin);
		def("buzz_q", _p->buzzer_quiet);
		def("auto_add", _p->autoadd_config);
		def("man_add", _p->manual_add_contacts);
		def("tel_base", _p->telemetry_mode_base);
		def("tel_loc", _p->telemetry_mode_loc);
		def("tel_env", _p->telemetry_mode_env);
		def("tz_offset", _p->tz_offset);
	}

public:
	explicit CompJson(NodePrefs *p) : _p(p) {}
};

class WifiJson : public ConfigSerializer {
	NodePrefs *_p;

protected:
	void structure() override
	{
		def("ssid", _p->wifi_ssid, sizeof(_p->wifi_ssid));
		def("pwd", _p->wifi_pwd, sizeof(_p->wifi_pwd));
		def("enabled", _p->wifi_enabled);
	}

public:
	explicit WifiJson(NodePrefs *p) : _p(p) {}
};

class ZcCompanionJson : public ZcCommonJson {
protected:
	void structure() override
	{
		ZcCommonJson::structure();
		def("ble_off", _p->ble_disabled);
		def("bright", _p->display_brightness);
		def("wake", _p->wake_on_msg);
		def("scr_off", _p->screen_off_secs);
		def("auto_off_mv", _p->auto_shutdown_mv);
		def("vc_en", _p->v_contact_enabled);
		def("vc_flags", _p->v_contact_flags);
		def("vbat_mv", _p->v_battery_alert_mv);
		def("adc_mult", _p->adc_multiplier);
	}

public:
	explicit ZcCompanionJson(NodePrefs *p) : ZcCommonJson(p) {}
};

class CompanionJson : public ConfigSerializer {
	NodePrefs *_p;
	RadioJson _radio;
	GpsJson _gps;
	RepeatDisableJson _repeat;
	CompJson _comp;
	WifiJson _wifi;
	ZcCompanionJson _zc;

protected:
	void structure() override
	{
		def("name", _p->node_name, sizeof(_p->node_name));
		def("lat", _p->node_lat);
		def("lon", _p->node_lon);
		def("radio", _radio);
		def("gps", _gps);
		def("repeat", _repeat);
		def("comp", _comp);
		def("wifi", _wifi);
		def("zc", _zc);
	}

public:
	explicit CompanionJson(NodePrefs *p)
		: _p(p), _radio(p), _gps(p), _comp(p), _wifi(p), _zc(p)
	{
		/* upstream: repeat.disable; ours: client_repeat (1 = forward) */
		_repeat.disable = p->client_repeat ? 0 : 1;
	}

	bool save(Stream &s) { return saveSerial(s); }
	bool load(Stream &s)
	{
		bool ok = loadSerial(s);

		_p->client_repeat = _repeat.disable ? 0 : 1;
		return ok;
	}
};

/* ── Servers: upstream helpers/CommonCLI.h ──────────────────────────── */

class RepeatJson : public ConfigSerializer {
	NodePrefs *_p;

protected:
	void structure() override
	{
		def("disable", _p->disable_fwd);
		def("f_max", _p->flood_max);
		def("f_max_uns", _p->flood_max_unscoped);
		def("f_max_adv", _p->flood_max_advert);
		def("loop", _p->loop_detect);
	}

public:
	explicit RepeatJson(NodePrefs *p) : _p(p) {}
};

class RoomJson : public ConfigSerializer {
	NodePrefs *_p;

protected:
	void structure() override { def("rd_only", _p->allow_read_only); }

public:
	explicit RoomJson(NodePrefs *p) : _p(p) {}
};

class PowerJson : public ConfigSerializer {
	NodePrefs *_p;

protected:
	void structure() override
	{
		def("adc_mult", _p->adc_multiplier);
		def("pwr_sav_en", _p->powersaving_enabled);
	}

public:
	explicit PowerJson(NodePrefs *p) : _p(p) {}
};

class ZcServerJson : public ZcCommonJson {
protected:
	void structure() override
	{
		ZcCommonJson::structure();
		def("tz_offset", _p->tz_offset);
		def("backoff", _p->backoff_multiplier);
		def("disc_ts", _p->discovery_mod_timestamp);
	}

public:
	explicit ZcServerJson(NodePrefs *p) : ZcCommonJson(p) {}
};

class ServerJson : public ConfigSerializer {
	NodePrefs *_p;
	RadioJson _radio;
	GpsJson _gps;
	RepeatJson _repeat;
	RoomJson _room;
	PowerJson _power;
	ZcServerJson _zc;

protected:
	void structure() override
	{
		def("name", _p->node_name, sizeof(_p->node_name));
		def("pass", _p->password, sizeof(_p->password));
		def("guest", _p->guest_password, sizeof(_p->guest_password));
		def("owner", _p->owner_info, sizeof(_p->owner_info));
		def("adv_int", _p->advert_interval);
		def("f_adv_int", _p->flood_advert_interval);
		def("lat", _p->node_lat);
		def("lon", _p->node_lon);
		def("radio", _radio);
		def("gps", _gps);
		def("repeat", _repeat);
		def("room", _room);
		def("power", _power);
		def("zc", _zc);
	}

public:
	explicit ServerJson(NodePrefs *p)
		: _p(p), _radio(p), _gps(p), _repeat(p), _room(p), _power(p), _zc(p) {}

	bool save(Stream &s) { return saveSerial(s); }
	bool load(Stream &s) { return loadSerial(s); }
};

void saneRadio(NodePrefs &p)
{
	if (p.freq < ZC_RADIO_FREQ_MIN_MHZ || p.freq > ZC_RADIO_FREQ_MAX_MHZ ||
	    p.sf < 5 || p.sf > 12 ||
	    p.bw < ZC_RADIO_BW_MIN_KHZ || p.bw > ZC_RADIO_BW_MAX_KHZ) {
		p.freq = mesh::LoRaConfig::FREQ_MHZ;
		p.bw = mesh::LoRaConfig::BANDWIDTH_KHZ;
		p.sf = mesh::LoRaConfig::SPREADING_FACTOR;
		p.cr = mesh::LoRaConfig::CODING_RATE;
		p.tx_power_dbm = mesh::LoRaConfig::TX_POWER_DBM;
	}
}

}  // namespace

/* Writing only reads the struct; the serializer API takes references. */

bool companionPrefsToJson(const NodePrefs &p, Stream &out)
{
	CompanionJson j(const_cast<NodePrefs *>(&p));

	return j.save(out);
}

bool serverPrefsToJson(const NodePrefs &p, Stream &out)
{
	ServerJson j(const_cast<NodePrefs *>(&p));

	return j.save(out);
}

bool companionPrefsFromJson(NodePrefs &p, Stream &in)
{
	NodePrefs t = p;
	CompanionJson j(&t);

	if (!j.load(in)) {
		return false;
	}
	saneRadio(t);
	sanitizeNodePrefs(&t);
	p = t;
	return true;
}

bool serverPrefsFromJson(NodePrefs &p, Stream &in)
{
	NodePrefs t = p;
	ServerJson j(&t);

	if (!j.load(in)) {
		return false;
	}
	saneRadio(t);
	sanitizeNodePrefs(&t);
	p = t;
	return true;
}
