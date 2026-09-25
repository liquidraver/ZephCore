/*
 * SPDX-License-Identifier: MIT
 * The companion's _serial: upstream's MultiSerialInterface, plus the one rule
 * ZephCore adds on top of it — a lossless reply is never lost to a full
 * transport.
 *
 * A reply (code < 0x80) the transports could not take is held here and
 * reported as sent, so callers that retry (contact and message sync) do not
 * send it twice. While it is held, nothing else is written and no command is
 * read: the app sees replies in order. retry() pushes it out once a transport
 * has room; drop() discards it when the session ends. Pushes (>= 0x80) stay
 * lossy. Main thread only.
 */

#pragma once

#include <string.h>
#include <helpers/BaseSerialInterface.h>

class CompanionSerial : public BaseSerialInterface {
	BaseSerialInterface &_inner;
	uint16_t _held_len = 0;  /* 0 = nothing held */
	uint8_t _held[MAX_FRAME_SIZE];

public:
	explicit CompanionSerial(BaseSerialInterface &inner) : _inner(inner) {}

	void enable() override { _inner.enable(); }
	void disable() override { _inner.disable(); }
	bool isEnabled() const override { return _inner.isEnabled(); }
	bool isConnected() const override { return _inner.isConnected(); }
	void loop() override { _inner.loop(); }
	bool isWriteBusy() const override { return _held_len != 0 || _inner.isWriteBusy(); }

	size_t writeFrame(const uint8_t src[], size_t len) override
	{
		if (len == 0 || len > MAX_FRAME_SIZE || _held_len != 0) {
			return 0;
		}
		/* Nobody to hold it for; holding would stall the next session */
		if (!_inner.isConnected()) {
			return 0;
		}
		if (_inner.writeFrame(src, len) == len) {
			return len;
		}
		if (src[0] < 0x80) {
			memcpy(_held, src, len);
			_held_len = (uint16_t)len;
			return len;
		}
		return 0;
	}

	size_t checkRecvFrame(uint8_t dest[]) override
	{
		return _held_len != 0 ? 0 : _inner.checkRecvFrame(dest);
	}

	/* Send the held reply if a transport has room now. */
	void retry()
	{
		if (_held_len == 0) {
			return;
		}
		if (!_inner.isConnected() || _inner.writeFrame(_held, _held_len) == _held_len) {
			_held_len = 0;
		}
	}

	void drop() { _held_len = 0; }
	bool holding() const { return _held_len != 0; }
};
