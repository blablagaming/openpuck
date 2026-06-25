// mode_lizard.h -- configurable lizard (desktop) keyboard+mouse mode.
//
// Not a standalone IController: lizard rides ON the puck interface (puck_hid.cpp calls rfLizard
// when Steam is closed or in MODE_LIZARD), driving mouse(0x40)+keyboard(0x41)+consumer(0x03) on
// the same puck HID slot. The mapping table g_lizardMap (lizard_map.h) drives all outputs; the
// default map mirrors the original hardcoded Valve SC1 behavior. Mouse reuses the Xbox-mode
// velocity+friction+sub-pixel glide model (g_mDiv / g_mFric).
#pragma once
#include <Adafruit_TinyUSB.h>
#include <stdint.h>

// mdev / kdev may be the same object (puck mode sends to the same HID slot for both reports).
void rfLizard(Adafruit_USBD_HID *mdev, Adafruit_USBD_HID *kdev, uint8_t mrid,
	      uint8_t krid);
