#include "mode_lizard.h"
#include "lizard_map.h"
#include "triton.h"
#include "config.h"
#include <string.h>

void rfLizard(Adafruit_USBD_HID *mdev, Adafruit_USBD_HID *kdev, uint8_t mrid,
	      uint8_t krid)
{
	const PuckInput &in = g_in[0];
	uint32_t buttons = in.buttons;

	// Append virtual stick-deflection bits so bindings can trigger on stick direction
	if (in.lx > 12000)
		buttons |= LZ_BTN_LSTICK_RT;
	if (in.lx < -12000)
		buttons |= LZ_BTN_LSTICK_LF;
	if (in.ly < -12000)
		buttons |= LZ_BTN_LSTICK_DN;
	if (in.ly > 12000)
		buttons |= LZ_BTN_LSTICK_UP;

	// ---- accumulate outputs from the binding table ----
	uint8_t outMod = 0;
	uint8_t outKeys[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t nKeys = 0;
	uint8_t outMBtn = 0;
	bool doRpadMouse = false, doLpadScroll = false;
	uint8_t consumerBits = 0;

	// kbdConsumed: any trig bits claimed by a hold-modifier binding so that simpler
	// single-button bindings sharing those bits are suppressed.
	uint32_t kbdConsumed = 0;

	for (uint8_t i = 0; i < g_lizardMap.count; i++) {
		const LizardBinding &b = g_lizardMap.bindings[i];
		if (b.outType == LZ_OUT_NONE)
			continue;

		if (b.outType == LZ_OUT_MOUSE_AXIS) {
			if (b.outData[0] == LZ_MSRC_RPAD)
				doRpadMouse = true;
			// LZ_MSRC_LSTICK and LZ_MSRC_GYRO could be added later
			continue;
		}
		if (b.outType == LZ_OUT_SCROLL) {
			if (b.outData[0] == LZ_MSRC_LPAD)
				doLpadScroll = true;
			continue;
		}

		// Check hold modifier (AND) then trigger (any-of OR)
		if (b.holdMask && (buttons & b.holdMask) != b.holdMask)
			continue;
		if (b.trigMask && (buttons & b.trigMask) == 0)
			continue;

		switch (b.outType) {
		case LZ_OUT_MOUSE_BTN:
			if (!b.trigMask || (buttons & b.trigMask))
				outMBtn |= b.outData[0];
			break;

		case LZ_OUT_KBD_CHORD:
			// suppress if trig bits already claimed by a hold-modifier binding
			if (b.holdMask == 0 && b.trigMask != 0 &&
			    (b.trigMask & kbdConsumed))
				break;
			outMod |= b.outData[0];
			for (int j = 1; j < 7 && b.outData[j]; j++) {
				if (nKeys < 6)
					outKeys[nKeys++] = b.outData[j];
			}
			// claim trig bits so simpler bindings don't double-fire
			if (b.holdMask != 0)
				kbdConsumed |= b.trigMask;
			break;

		case LZ_OUT_CONSUMER:
			if (g_usbMode == MODE_LIZARD)
				consumerBits |= b.outData[0];
			break;

		default:
			break;
		}
	}

	// ---- right pad -> mouse motion with glide ----
	static int prx = 0, pry = 0;
	static bool prt = false;
	static float vx = 0, vy = 0, rmx = 0, rmy = 0;
	int dx = 0, dy = 0;
	if (doRpadMouse) {
		bool rtouch = (buttons & TB_RPADT) != 0;
		int rx = in.rpx, ry = in.rpy;
		if (rtouch && prt) {
			vx += (rx - prx);
			vy += (ry - pry);
		}
		if (rtouch) {
			prx = rx;
			pry = ry;
		}
		prt = rtouch;
		// Y inverted; *10 = desktop cursor sensitivity (g_mDiv 64 -> eff 640)
		float mxf = vx / (float)(g_mDiv * 10) + rmx,
		      myf = -(vy / (float)(g_mDiv * 10)) + rmy;
		dx = (int)mxf;
		dy = (int)myf;
		rmx = mxf - dx;
		rmy = myf - dy; // sub-pixel carry
		if (dx > 127)
			dx = 127;
		if (dx < -127)
			dx = -127;
		if (dy > 127)
			dy = 127;
		if (dy < -127)
			dy = -127;
		float f = g_mFric / 100.0f;
		vx *= f;
		vy *= f;
		if (vx > -1 && vx < 1)
			vx = 0;
		if (vy > -1 && vy < 1)
			vy = 0;
	}

	// ---- left pad -> scroll wheel ----
	static int ply = 0;
	static bool plt = false;
	static float sacc = 0;
	int dw = 0;
	if (doLpadScroll) {
		bool ltouch = (buttons & TB_LPADT) != 0;
		int ly = in.lpy;
		if (ltouch && plt)
			sacc += (ly - ply) / (float)(g_mDiv * 24);
		if (ltouch)
			ply = ly;
		else
			sacc = 0;
		plt = ltouch;
		dw = (int)sacc;
		sacc -= dw;
		if (dw > 15)
			dw = 15;
		if (dw < -15)
			dw = -15;
	}

	// ---- mouse report ----
	static uint8_t pmbtn = 0;
	if (dx || dy || dw || outMBtn != pmbtn) {
		pmbtn = outMBtn;
		hid_mouse_report_t m;
		m.buttons = outMBtn;
		m.x = (int8_t)dx;
		m.y = (int8_t)dy;
		m.wheel = (int8_t)dw;
		m.pan = 0;
		if (mdev->ready())
			mdev->sendReport(mrid, &m, sizeof m);
	}

	// ---- consumer control (edge-triggered) ----
	static uint8_t prevCC = 0;
	if (g_usbMode == MODE_LIZARD) {
		if (consumerBits != prevCC) {
			if (mdev->ready())
				mdev->sendReport(0x03, &consumerBits, 1);
			prevCC = consumerBits;
		}
	}

	// ---- keyboard report ----
	static uint8_t pmod = 0, pkc[6] = { 0, 0, 0, 0, 0, 0 };
	bool chg = (outMod != pmod);
	for (int i = 0; i < 6; i++)
		if (outKeys[i] != pkc[i])
			chg = true;
	if (chg) {
		pmod = outMod;
		for (int i = 0; i < 6; i++)
			pkc[i] = outKeys[i];
		uint8_t krep[8] = { outMod,    0,	    outKeys[0],
				    outKeys[1], outKeys[2], outKeys[3],
				    outKeys[4], outKeys[5] };
		if (kdev->ready())
			kdev->sendReport(krid, krep, 8);
	}
}
