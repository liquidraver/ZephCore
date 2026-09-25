/*
 * SPDX-License-Identifier: MIT
 * Arduino Streams over an open Zephyr file, for ConfigSerializer (prefs.json).
 *
 * The parser calls available() and read() once per character, so the reader
 * refills a small buffer instead of asking the filesystem each time. The
 * writer remembers any short write so the atomic replace can refuse to commit.
 */

#pragma once

#include <Stream.h>
#include <zephyr/fs/fs.h>

class FsReadStream : public Stream {
	struct fs_file_t *_f;
	uint8_t _buf[64];
	size_t _len = 0;
	size_t _pos = 0;
	bool _eof = false;

	bool fill()
	{
		if (_pos < _len) {
			return true;
		}
		if (_eof) {
			return false;
		}
		ssize_t n = fs_read(_f, _buf, sizeof(_buf));

		if (n <= 0) {
			_eof = true;
			return false;
		}
		_len = (size_t)n;
		_pos = 0;
		return true;
	}

public:
	explicit FsReadStream(struct fs_file_t *f) : _f(f) {}

	int available() override { return fill() ? (int)(_len - _pos) : 0; }
	int read() override { return fill() ? _buf[_pos++] : -1; }
	int peek() override { return fill() ? _buf[_pos] : -1; }
	size_t write(uint8_t) override { return 0; }
};

class FsWriteStream : public Stream {
	struct fs_file_t *_f;
	bool _ok = true;

public:
	explicit FsWriteStream(struct fs_file_t *f) : _f(f) {}

	bool ok() const { return _ok; }

	size_t write(uint8_t c) override { return write(&c, 1); }
	size_t write(const uint8_t *buf, size_t size) override
	{
		ssize_t n = fs_write(_f, buf, size);

		if (n < 0 || (size_t)n != size) {
			_ok = false;
			return n > 0 ? (size_t)n : 0;
		}
		return size;
	}
	int available() override { return 0; }
	int read() override { return -1; }
	int peek() override { return -1; }
};
