#include "report45.h"
#include "triton.h"
#include <string.h>

// Write a signed 16-bit value little-endian at rep[2+off] (the same indexing triton.h's s16off reads).
static inline void put_s16(uint8_t *rep, int off, int16_t v)
{
	rep[2 + off] = (uint8_t)(v & 0xFF);
	rep[2 + off + 1] = (uint8_t)((v >> 8) & 0xFF);
}
// Write an unsigned 16-bit value little-endian at rep[2+off] (matches u16off).
static inline void put_u16(uint8_t *rep, int off, uint16_t v)
{
	rep[2 + off] = (uint8_t)(v & 0xFF);
	rep[2 + off + 1] = (uint8_t)((v >> 8) & 0xFF);
}

void buildReport45(uint8_t *out, uint8_t seq)
{
	memset(out, 0, REPORT45_LEN);
	out[0] = 0x45;
	out[1] = seq;

	// buttons u32 at rep[2..5] (inverse of btnsOf)
	out[2] = (uint8_t)(g_in.buttons & 0xFF);
	out[3] = (uint8_t)((g_in.buttons >> 8) & 0xFF);
	out[4] = (uint8_t)((g_in.buttons >> 16) & 0xFF);
	out[5] = (uint8_t)((g_in.buttons >> 24) & 0xFF);

	// triggers: the controller's analog trigger is a u16 that trigU8 maps via >>7 (saturating). Inverse:
	// u16 = lt << 7 (so a full 0xFF pull -> 0x7F80 -> trigU8 -> 0xFF). offset 4 = LT, offset 6 = RT.
	put_u16(out, 4, (uint16_t)(g_in.lt << 7));
	put_u16(out, 6, (uint16_t)(g_in.rt << 7));

	// sticks (int16, center 0): lx@8 ly@10 rx@12 ry@14
	put_s16(out, 8, g_in.lx);
	put_s16(out, 10, g_in.ly);
	put_s16(out, 12, g_in.rx);
	put_s16(out, 14, g_in.ry);

	// trackpads: lpx@16 lpy@18 rpx@22 rpy@24 (offsets 20/26 unused by the decode)
	put_s16(out, 16, g_in.lpx);
	put_s16(out, 18, g_in.lpy);
	put_s16(out, 22, g_in.rpx);
	put_s16(out, 24, g_in.rpy);

	// IMU: accel @0x22 (offset 32), gyro @0x28 (offset 38) -- inverse of imuFrom45
	put_s16(out, 32, g_in.ax);
	put_s16(out, 34, g_in.ay);
	put_s16(out, 36, g_in.az);
	put_s16(out, 38, g_in.gx);
	put_s16(out, 40, g_in.gy);
	put_s16(out, 42, g_in.gz);
}
