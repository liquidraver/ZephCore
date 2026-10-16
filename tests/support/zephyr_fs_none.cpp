// Host boundary for the mesh-level zcpfs tests: helpers/compat/ZephyrFS.h with
// no file system behind it. ClientACL.cpp links against it; the tests never
// load or save an access list, and every call fails as on a missing file.
#include <ZephyrFS.h>

bool File::openPath(const char *, fs_mode_t) { return false; }
size_t File::read(uint8_t *, size_t) { return 0; }
size_t File::write(const uint8_t *, size_t) { return 0; }
int File::read() { return -1; }
int File::peek() { return -1; }
int File::available() { return 0; }
void File::close() { _open = false; }

void ZephyrFS::fullPath(char *dest, size_t dest_len, const char *) const { if (dest_len) dest[0] = 0; }
bool ZephyrFS::exists(const char *) { return false; }
File ZephyrFS::open(const char *, const char *, bool) { return File(); }
bool ZephyrFS::remove(const char *) { return false; }
bool ZephyrFS::mkdir(const char *) { return false; }
