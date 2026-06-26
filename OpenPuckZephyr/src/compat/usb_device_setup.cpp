// usb_device_setup.cpp -- the Zephyr usbd_context for the OpenPuck port.
//
// Builds one full-speed device, registers all class instances picked up from
// devicetree (the HID pool in app.overlay + the board CDC ACM console), and
// brings the stack up. The TinyUSB shim's USBDevice.attach()/detach() call
// opk_usbd_enable()/disable(); the per-mode VID/PID/bcdDevice the firmware set
// through the shim (g_opk_usb_id) are applied here just before enable.
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include "Adafruit_TinyUSB.h"
#include "usb_identity.h"

LOG_MODULE_REGISTER(opk_usbd, LOG_LEVEL_INF);

extern "C" void opk_usb_msg(const enum usbd_msg_type type);

USBD_DEVICE_DEFINE(opk_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), 0x28DE,
		   0x1304);

USBD_DESC_LANG_DEFINE(opk_lang);
USBD_DESC_MANUFACTURER_DEFINE(opk_mfr, "OpenPuck");
USBD_DESC_PRODUCT_DEFINE(opk_product, "OpenPuck");
USBD_DESC_SERIAL_NUMBER_DEFINE(opk_sn);

USBD_DESC_CONFIG_DEFINE(opk_fs_cfg_desc, "OpenPuck FS");

// Bus-powered + remote-wakeup (the firmware always arms remote wakeup).
static const uint8_t opk_attr = USB_SCD_REMOTE_WAKEUP;
USBD_CONFIGURATION_DEFINE(opk_fs_config, opk_attr, 250, &opk_fs_cfg_desc);

static bool s_setup_done;
static bool s_enabled;

static void msg_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
	(void)ctx;
	opk_usb_msg(msg->type);
}

static int do_setup(void)
{
	int err;

	if (s_setup_done)
		return 0;

	if ((err = usbd_add_descriptor(&opk_usbd, &opk_lang)) ||
	    (err = usbd_add_descriptor(&opk_usbd, &opk_mfr)) ||
	    (err = usbd_add_descriptor(&opk_usbd, &opk_product)) ||
	    (err = usbd_add_descriptor(&opk_usbd, &opk_sn))) {
		LOG_ERR("descriptor add failed (%d)", err);
		return err;
	}

	err = usbd_add_configuration(&opk_usbd, USBD_SPEED_FS, &opk_fs_config);
	if (err) {
		LOG_ERR("add configuration failed (%d)", err);
		return err;
	}

	// Register every class instance from devicetree (HID pool + board CDC ACM).
	err = usbd_register_all_classes(&opk_usbd, USBD_SPEED_FS, 1, NULL);
	if (err) {
		LOG_ERR("register classes failed (%d)", err);
		return err;
	}

	usbd_device_set_code_triple(&opk_usbd, USBD_SPEED_FS, 0, 0, 0);
	usbd_msg_register_cb(&opk_usbd, msg_cb);
	s_setup_done = true;
	return 0;
}

extern "C" struct usbd_context *opk_usbd_ctx(void)
{
	return &opk_usbd;
}

extern "C" int opk_usbd_enable(void)
{
	int err = do_setup();
	if (err)
		return err;

	// Apply the per-mode identity the firmware staged through the shim.
	usbd_device_set_vid(&opk_usbd, g_opk_usb_id.vid);
	usbd_device_set_pid(&opk_usbd, g_opk_usb_id.pid);
	usbd_device_set_bcd_device(&opk_usbd, g_opk_usb_id.bcd_device);

	if (s_enabled)
		return 0;
	err = usbd_init(&opk_usbd);
	if (err && err != -EALREADY) {
		LOG_ERR("usbd_init failed (%d)", err);
		return err;
	}
	err = usbd_enable(&opk_usbd);
	if (err && err != -EALREADY) {
		LOG_ERR("usbd_enable failed (%d)", err);
		return err;
	}
	s_enabled = true;
	return 0;
}

extern "C" int opk_usbd_disable(void)
{
	if (!s_enabled)
		return 0;
	int err = usbd_disable(&opk_usbd);
	s_enabled = false;
	return err;
}
