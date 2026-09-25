/*
 * SPDX-License-Identifier: MIT
 * Arduino-style FILESYSTEM / File over the Zephyr fs API, so storage code
 * ported verbatim from upstream MeshCore compiles unchanged. A ZephyrFS is
 * rooted at a directory: upstream's "/regions2" opens <root>/regions2.
 */

#pragma once

#include "Stream.h"
#include <zephyr/fs/fs.h>

class File : public Stream {
	struct fs_file_t _file;
	bool _open;

public:
	File() : _open(false) { fs_file_t_init(&_file); }
	File(File &&other) : _file(other._file), _open(other._open) { other._open = false; }
	File(const File &) = delete;
	File &operator=(const File &) = delete;
	~File() { close(); }

	bool openPath(const char *path, fs_mode_t flags);

	explicit operator bool() const { return _open; }

	size_t read(uint8_t *buf, size_t size);
	size_t write(const uint8_t *buf, size_t size) override;
	size_t write(uint8_t c) override { return write(&c, 1); }
	using Print::write;

	int read() override;
	int peek() override;
	int available() override;
	void close();
};

class ZephyrFS {
	const char *_root;

	void fullPath(char *dest, size_t dest_len, const char *path) const;

public:
	explicit ZephyrFS(const char *root) : _root(root) {}

	bool exists(const char *path);
	/* "r" (default): read. "w": truncate or create, then write. */
	File open(const char *path, const char *mode = "r", bool create = false);
	bool remove(const char *path);
	bool mkdir(const char *path);
};
