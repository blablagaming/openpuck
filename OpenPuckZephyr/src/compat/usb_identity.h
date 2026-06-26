// usb_identity.h -- USB identity/state recorded by the TinyUSB shim mutators and
// consumed when the Zephyr usbd_context descriptor set is built.
#pragma once
#include <stdint.h>

struct opk_usb_identity {
	uint16_t vid = 0x28DE; // Valve, default (puck identity)
	uint16_t pid = 0x1304;
	uint16_t bcd_usb = 0x0200;
	uint16_t bcd_device = 0x0100;
	const char *manufacturer = "OpenPuck";
	const char *product = "OpenPuck";
	const char *serial = "0";
	uint8_t cfg_attr = 0xA0; // bus-powered + remote-wakeup
	uint8_t next_itf = 0;
	uint8_t next_ep = 0;
};
extern opk_usb_identity g_opk_usb_id;
