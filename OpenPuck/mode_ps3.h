// mode_ps3.h -- Sony DualShock 3 personality (MODE_PS3): 054C:0289 + gyro + haptics.
//
// Presents the device as a DualShock 3 (PID 0x0289, with vibration) so it enumerates
// cleanly on a PS3 console and PS3-aware PC drivers. One HID slot (maxSlots=1): the PS3
// expects a single gamepad per USB port. Input streamed at ~250Hz from task(): sticks,
// digital buttons, analog button pressures, and the SC2 IMU mapped to the DS3's 3-axis
// accelerometer (unsigned big-endian, center 512) + single-axis gyro (yaw, same format).
// Haptic relay handles the PS3's rumble OUTPUT report (report 0x01, bytes 2 and 4).
// Feature reports 0xF2 and 0xF5 satisfy the PS3's DS3 identification handshake.
#pragma once
#include "controllers.h"

class Ps3Controller : public IController {
    public:
	void begin() override;
	void task() override;
	bool dynamicMount() const override
	{
		return true;
	}
	uint8_t maxSlots() const override;
	void usbIdentity() override;
	void beginPool() override;
	void mountSlots(uint8_t k) override;
};
extern Ps3Controller g_ps3Ctl;
