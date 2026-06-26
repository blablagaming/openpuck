// device/usbd_pvt.h -- compat shim for TinyUSB's private class-driver API.
//
// The XInput personality (mode_xinput.cpp) implements a custom TinyUSB class
// driver. Zephyr's USBD uses a different class-driver model, so XInput needs a
// real Zephyr custom-class port to enumerate; until then this header provides
// the types/prototypes so mode_xinput.cpp compiles and its report-building logic
// stays intact. The endpoint primitives are stubbed in usb_stub.cpp.
#pragma once
#include <stdint.h>
#include "../Adafruit_TinyUSB.h"

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

typedef enum {
	XFER_RESULT_SUCCESS = 0,
	XFER_RESULT_FAILED,
	XFER_RESULT_STALLED,
	XFER_RESULT_TIMEOUT,
	XFER_RESULT_INVALID,
} xfer_result_t;

typedef enum {
	CONTROL_STAGE_IDLE = 0,
	CONTROL_STAGE_SETUP,
	CONTROL_STAGE_DATA,
	CONTROL_STAGE_ACK,
} control_stage_t;

// TinyUSB endpoint-address direction helper.
static inline uint8_t tu_edpt_dir(uint8_t addr)
{
	return (addr & 0x80) ? TUSB_DIR_IN : TUSB_DIR_OUT;
}

// TinyUSB class-driver vtable (subset the XInput driver initializes).
typedef struct {
#if CFG_TUSB_DEBUG >= 2
	const char *name;
#endif
	void (*init)(void);
	bool (*deinit)(void);
	void (*reset)(uint8_t rhport);
	uint16_t (*open)(uint8_t rhport, const tusb_desc_interface_t *itf_desc,
			 uint16_t max_len);
	bool (*control_xfer_cb)(uint8_t rhport, uint8_t stage,
				const tusb_control_request_t *request);
	bool (*xfer_cb)(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
			uint32_t xferred_bytes);
	void (*sof)(uint8_t rhport);
} usbd_class_driver_t;

#ifdef __cplusplus
extern "C" {
#endif
bool tud_mounted(void);
bool usbd_edpt_open(uint8_t rhport, const tusb_desc_endpoint_t *ep);
bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep, uint8_t *buf, uint16_t n);
bool usbd_edpt_busy(uint8_t rhport, uint8_t ep);
bool usbd_edpt_claim(uint8_t rhport, uint8_t ep);
void usbd_edpt_release(uint8_t rhport, uint8_t ep);
#ifdef __cplusplus
}
#endif
