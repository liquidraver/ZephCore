#include "test.h"
#include <algorithm>
#include <cerrno>
#include <cstring>

namespace {
// The generated include contains production functions from patch 0003. Only SPI,
// the retention-list API and warm sleep are modeled here, not driver scheduling.
constexpr uint16_t SX126X_REG_RX_GAIN = 0x08AC;
constexpr uint16_t SX126X_REG_TX_MODULATION = 0x0889;
constexpr uint16_t SX126X_REG_RETENTION_LIST = 0x029F;
constexpr int SX126X_MAX_RETENTION_REGS = 4;
struct sx126x_data {
	bool rx_boost_enabled = true, awake = true, bad_third = false;
	uint8_t heltec = 0x04;
	unsigned retention_checks = 0, retention_fails = 0, warnings = 0;
	unsigned heltec_reads = 0, heltec_writes = 0;
	int read_error = 0, write_error = 0, add_error = 0;
	uint16_t fail_address = 0x08B5;
	std::vector<uint16_t> retained;
};
struct device { sx126x_data *data; };
uint16_t sys_get_be16(const uint8_t *p) { return uint16_t(p[0] << 8) | p[1]; }
uint8_t sx126x_rx_gain_byte(const device *, bool boost) { return boost ? 0x96 : 0x94; }
template<typename... Args> void log_boundary(Args...) {}
#define LOG_WRN(...) (++dev->data->warnings, log_boundary(__VA_ARGS__))
#define LOG_INF(...) log_boundary(__VA_ARGS__)
#define IS_ENABLED(option) (option)

int sx126x_hal_read_regs(const device *dev, uint16_t address, uint8_t *out, int size)
{
	auto &d = *dev->data;
	if (!d.awake) return -EAGAIN;
	if (address == 0x08B5) {
		++d.heltec_reads;
		if (d.read_error) return d.read_error;
		CHECK(size == 1); *out = d.heltec;
	} else if (address == SX126X_REG_RETENTION_LIST) {
		CHECK(size == 9); std::memset(out, 0, size);
		out[0] = uint8_t(d.retained.size());
		for (size_t i = 0; i < d.retained.size(); ++i) {
			out[1 + 2*i] = uint8_t(d.retained[i] >> 8);
			out[2 + 2*i] = uint8_t(d.retained[i]);
		}
		if (d.bad_third) out[6] ^= 1;
	} else {
		CHECK(address == SX126X_REG_RX_GAIN && size == 1); *out = 0x96;
	}
	return 0;
}
int sx126x_hal_write_regs(const device *dev, uint16_t address, const uint8_t *in, int size)
{
	auto &d = *dev->data;
	CHECK(address == 0x08B5 && size == 1);
	++d.heltec_writes;
	if (!d.awake) return -EAGAIN;
	if (d.write_error) return d.write_error;
	d.heltec = *in;
	return 0;
}
int sx126x_add_reg_to_retention(const device *dev, uint16_t address)
{
	auto &d = *dev->data;
	if (address == d.fail_address && d.add_error) return d.add_error;
	if (std::find(d.retained.begin(), d.retained.end(), address) == d.retained.end()) {
		if (d.retained.size() == 4) return -ENOSPC;
		d.retained.push_back(address);
	}
	return 0;
}
void warm_sleep(sx126x_data &d)
{
	d.awake = false;
	if (std::find(d.retained.begin(), d.retained.end(), 0x08B5) == d.retained.end())
		d.heltec = 0x04;
	d.awake = true;
}
namespace enabled {
#define CONFIG_ZEPHCORE_SX126X_HELTEC_REG_PATCH 1
#include "sx126x_helpers.inc"
#undef CONFIG_ZEPHCORE_SX126X_HELTEC_REG_PATCH
}
namespace disabled {
#define CONFIG_ZEPHCORE_SX126X_HELTEC_REG_PATCH 0
#include "sx126x_helpers.inc"
#undef CONFIG_ZEPHCORE_SX126X_HELTEC_REG_PATCH
}
#undef IS_ENABLED
#undef LOG_INF
#undef LOG_WRN
} // namespace

TEST(heltec_bit_and_spi_errors, "UNIT-HELTEC-001", "Heltec bit preserves other bits and propagates SPI failures") {
	for (uint8_t initial : {0x00, 0x04, 0x05, 0xA4, 0xFE, 0xFF}) {
		sx126x_data d; device dev{&d}; d.heltec = initial;
		CHECK(enabled::sx126x_apply_heltec_rx_patch(&dev) == 0);
		CHECK(d.heltec == uint8_t(initial | 1));
	}
	sx126x_data d; device dev{&d}; d.read_error = -EIO;
	CHECK(enabled::sx126x_apply_heltec_rx_patch(&dev) == -EIO);
	CHECK(d.heltec_writes == 0 && d.heltec == 0x04);
	d.read_error = 0; d.write_error = -ETIMEDOUT;
	CHECK(enabled::sx126x_apply_heltec_rx_patch(&dev) == -ETIMEDOUT);
	CHECK(d.heltec == 0x04);
	d.awake = false;
	CHECK(enabled::sx126x_apply_heltec_rx_patch(&dev) == -EAGAIN);
}
TEST(heltec_sleep_and_reapply, "UNIT-HELTEC-002", "Heltec workaround survives warm sleep and calibration-style reapply") {
	sx126x_data d; device dev{&d};
	for (int i = 0; i < 10; ++i) {
		CHECK(enabled::sx126x_install_retention_list(&dev) == 0);
		warm_sleep(d); CHECK(d.heltec == 0x05);
		CHECK(d.retained == std::vector<uint16_t>({0x08AC, 0x0889, 0x08B5}));
	}
	// Model calibration disturbing the workaround and list, then run real restore code.
	d.heltec = 0x04; d.retained.clear();
	CHECK(enabled::sx126x_install_retention_list(&dev) == 0);
	warm_sleep(d); CHECK(d.heltec == 0x05 && d.warnings == 0);
}
TEST(heltec_capability_off, "UNIT-HELTEC-003", "Capability off leaves Heltec register untouched and retains two entries") {
	sx126x_data d; device dev{&d};
	CHECK(disabled::sx126x_apply_heltec_rx_patch(&dev) == 0);
	CHECK(disabled::sx126x_install_retention_list(&dev) == 0);
	CHECK(d.heltec_reads == 0 && d.heltec_writes == 0 && d.heltec == 0x04);
	CHECK(d.retained == std::vector<uint16_t>({0x08AC, 0x0889}) && d.warnings == 0);
}
TEST(heltec_retention_errors, "UNIT-HELTEC-004", "Retention errors propagate and debug check validates the third entry") {
	for (uint16_t address : {0x08AC, 0x0889, 0x08B5}) {
		sx126x_data d; device dev{&d}; d.fail_address = address; d.add_error = -ENOSPC;
		CHECK(enabled::sx126x_install_retention_list(&dev) == -ENOSPC);
		CHECK(d.heltec_writes == 0);
	}
	sx126x_data d; device dev{&d}; d.write_error = -EIO;
	CHECK(enabled::sx126x_install_retention_list(&dev) == -EIO);
	d.write_error = 0;
	CHECK(enabled::sx126x_install_retention_list(&dev) == 0);
	enabled::sx126x_retention_debug_check(&dev, "valid", true);
	CHECK(d.retention_checks == 1 && d.retention_fails == 0 && d.warnings == 0);
	d.bad_third = true;
	enabled::sx126x_retention_debug_check(&dev, "corrupt", true);
	CHECK(d.retention_fails == 1 && d.warnings == 1);
}
