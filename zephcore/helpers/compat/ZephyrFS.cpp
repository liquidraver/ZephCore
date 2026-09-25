/*
 * SPDX-License-Identifier: MIT
 */

#include "ZephyrFS.h"
#include <stdio.h>
#include <string.h>

bool File::openPath(const char *path, fs_mode_t flags)
{
	close();
	fs_file_t_init(&_file);
	_open = fs_open(&_file, path, flags) == 0;
	return _open;
}

size_t File::read(uint8_t *buf, size_t size)
{
	if (!_open) {
		return 0;
	}
	ssize_t n = fs_read(&_file, buf, size);
	return n > 0 ? (size_t)n : 0;
}

size_t File::write(const uint8_t *buf, size_t size)
{
	if (!_open) {
		return 0;
	}
	ssize_t n = fs_write(&_file, buf, size);
	return n > 0 ? (size_t)n : 0;
}

int File::read()
{
	uint8_t c;
	return read(&c, 1) == 1 ? c : -1;
}

int File::peek()
{
	int c = read();
	if (c >= 0) {
		fs_seek(&_file, -1, FS_SEEK_CUR);
	}
	return c;
}

int File::available()
{
	if (!_open) {
		return 0;
	}
	off_t pos = fs_tell(&_file);
	if (pos < 0 || fs_seek(&_file, 0, FS_SEEK_END) < 0) {
		return 0;
	}
	off_t end = fs_tell(&_file);
	fs_seek(&_file, pos, FS_SEEK_SET);
	return end > pos ? (int)(end - pos) : 0;
}

void File::close()
{
	if (_open) {
		fs_close(&_file);
		_open = false;
	}
}

void ZephyrFS::fullPath(char *dest, size_t dest_len, const char *path) const
{
	snprintf(dest, dest_len, "%s%s%s", _root, path[0] == '/' ? "" : "/", path);
}

bool ZephyrFS::exists(const char *path)
{
	char full[64];
	fullPath(full, sizeof(full), path);
	struct fs_dirent entry;
	return fs_stat(full, &entry) == 0;
}

File ZephyrFS::open(const char *path, const char *mode, bool create)
{
	char full[64];
	fullPath(full, sizeof(full), path);
	File file;
	if (mode[0] == 'w') {
		fs_unlink(full);  /* truncate: LittleFS rewrites a file in place otherwise */
		file.openPath(full, FS_O_CREATE | FS_O_WRITE);
	} else {
		file.openPath(full, FS_O_READ);
	}
	(void)create;
	return file;
}

bool ZephyrFS::remove(const char *path)
{
	char full[64];
	fullPath(full, sizeof(full), path);
	return fs_unlink(full) == 0;
}

bool ZephyrFS::mkdir(const char *path)
{
	char full[64];
	fullPath(full, sizeof(full), path);
	return fs_mkdir(full) == 0;
}
