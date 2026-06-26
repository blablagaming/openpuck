// Adafruit_LittleFS.h -- compat shim over Zephyr's LittleFS (fs subsystem).
//
// The persistence code (config.cpp, bonds.cpp, mode_switch_pro.cpp) was written
// against Adafruit_LittleFS's File API on the nRF52 internal flash. This shim
// reproduces just the surface they use, backed by a Zephyr littlefs mount on the
// board's storage_partition. Semantics match: open/read/write/close, plus
// remove()/format() on the filesystem object.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace Adafruit_LittleFS_Namespace
{

enum FileMode {
	FILE_O_READ = 0,
	FILE_O_WRITE = 1,
};

class LittleFS; // fwd

// One open file. Mirrors Adafruit's File: constructed from the FS object, then
// open()ed. read() returns bytes read (int, -1 style errors clamped to 0),
// write() returns bytes written.
class File
{
    public:
	explicit File(LittleFS &fs);
	~File();

	bool open(const char *path, uint8_t mode);
	int read(uint8_t *buf, size_t n);
	size_t write(const uint8_t *buf, size_t n);
	void close();
	explicit operator bool() const
	{
		return _open;
	}

    private:
	void *_fp; // struct fs_file_t* (opaque to keep Zephyr headers out of here)
	bool _open;
};

// The filesystem object. begin() mounts (idempotent), remove() deletes a file,
// format() reformats the partition (factory wipe).
class LittleFS
{
    public:
	bool begin();
	bool remove(const char *path);
	bool format();
};

} // namespace Adafruit_LittleFS_Namespace
