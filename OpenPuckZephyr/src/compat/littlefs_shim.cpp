// littlefs_shim.cpp -- backs the Adafruit_LittleFS / InternalFileSystem shim
// with Zephyr's LittleFS mounted on the board storage_partition.
#include "Adafruit_LittleFS.h"
#include "InternalFileSystem.h"

#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kernel.h>
#include <string.h>

using namespace Adafruit_LittleFS_Namespace;

#define LFS_MNT "/lfs"
#define PART_ID FIXED_PARTITION_ID(storage_partition)

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(opk_lfs_cfg);

static struct fs_mount_t opk_mp;
static bool opk_mounted;
static bool opk_mp_init;

static void mp_setup(void)
{
	if (opk_mp_init)
		return;
	opk_mp_init = true;
	opk_mp.type = FS_LITTLEFS;
	opk_mp.fs_data = &opk_lfs_cfg;
	opk_mp.storage_dev = (void *)PART_ID;
	opk_mp.mnt_point = LFS_MNT;
}

// Build the mount-relative full path. All callers pass leading-slash names.
static void fullpath(char *out, size_t n, const char *path)
{
	snprintf(out, n, "%s%s%s", LFS_MNT, path[0] == '/' ? "" : "/", path);
}

// ---- LittleFS ----
bool LittleFS::begin()
{
	mp_setup();
	if (opk_mounted)
		return true;
	int rc = fs_mount(&opk_mp);
	// littlefs auto-formats a blank/corrupt partition on mount, so a clean
	// device comes up empty (the Arduino InternalFS.begin() behaved the same).
	opk_mounted = (rc == 0);
	return opk_mounted;
}

bool LittleFS::remove(const char *path)
{
	char fp[64];
	fullpath(fp, sizeof fp, path);
	int rc = fs_unlink(fp);
	return rc == 0 || rc == -ENOENT;
}

bool LittleFS::format()
{
	if (opk_mounted) {
		fs_unmount(&opk_mp);
		opk_mounted = false;
	}
	int rc = fs_mkfs(FS_LITTLEFS, (uintptr_t)PART_ID, &opk_lfs_cfg, 0);
	if (rc < 0)
		return false;
	return begin();
}

// ---- File ----
File::File(LittleFS &fs) : _fp(nullptr), _open(false)
{
	(void)fs;
	_fp = k_malloc(sizeof(struct fs_file_t));
	if (_fp)
		fs_file_t_init((struct fs_file_t *)_fp);
}

File::~File()
{
	close();
	if (_fp)
		k_free(_fp);
}

bool File::open(const char *path, uint8_t mode)
{
	if (!_fp)
		return false;
	close();
	char fp[64];
	fullpath(fp, sizeof fp, path);
	fs_mode_t flags = (mode == FILE_O_WRITE) ?
				  (FS_O_CREATE | FS_O_WRITE) :
				  FS_O_READ;
	int rc = fs_open((struct fs_file_t *)_fp, fp, flags);
	_open = (rc == 0);
	// Zephyr has no FS_O_TRUNC; emulate Adafruit's write-truncates behavior so
	// a rewritten record never keeps a stale tail (callers also remove() first
	// for cfg/bonds, but swProSaveCfg overwrites in place).
	if (_open && mode == FILE_O_WRITE)
		fs_truncate((struct fs_file_t *)_fp, 0);
	return _open;
}

int File::read(uint8_t *buf, size_t n)
{
	if (!_open)
		return 0;
	ssize_t rc = fs_read((struct fs_file_t *)_fp, buf, n);
	return rc < 0 ? 0 : (int)rc;
}

size_t File::write(const uint8_t *buf, size_t n)
{
	if (!_open)
		return 0;
	ssize_t rc = fs_write((struct fs_file_t *)_fp, buf, n);
	return rc < 0 ? 0 : (size_t)rc;
}

void File::close()
{
	if (_open) {
		fs_close((struct fs_file_t *)_fp);
		_open = false;
	}
}

Adafruit_LittleFS_Namespace::LittleFS InternalFS;
