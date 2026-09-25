/*
 * SPDX-License-Identifier: MIT
 * Arduino's `Serial`, for code ported verbatim from upstream MeshCore: the
 * server roles' USB console. Output only.
 */

#pragma once

#include "Stream.h"

class ConsoleSerial : public Stream {
public:
	using Print::write;
	size_t write(uint8_t c) override { return write(&c, 1); }
	size_t write(const uint8_t *buffer, size_t size) override;

	int available() override { return 0; }
	int read() override { return -1; }
	int peek() override { return -1; }
};

/* Defined by the server roles' composition root (server_main_common.cpp). */
extern ConsoleSerial Serial;
