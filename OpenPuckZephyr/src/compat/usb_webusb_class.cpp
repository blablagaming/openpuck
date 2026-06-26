// usb_webusb_class.cpp -- Zephyr custom vendor class backing the WebUSB shim.
//
// Implements the single WebUSB vendor interface (1 bulk OUT + 1 bulk IN, FS) and
// bridges it to byte rings so the firmware's Adafruit_USBD_WebUSB API works:
//   read()/available()  <- OUT endpoint  (browser -> device)
//   write()/flush()      -> IN endpoint   (device -> browser)
// The BOS WebUSB platform-capability + URL request and the MS OS 2.0 descriptor
// (for WinUSB binding) are registered in usb_device_setup.cpp.
//
// Modeled on Zephyr's samples/subsys/usb/webusb. nRF52840 is full-speed only.
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(opk_webusb, LOG_LEVEL_ERR);

#define WUSB_ENABLED 0

NET_BUF_POOL_FIXED_DEFINE(wusb_pool, 4, 64, sizeof(struct udc_buf_info), NULL);

RING_BUF_DECLARE(wusb_rx_ring, 256);
static atomic_t wusb_state;

struct wusb_desc {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_out_ep;
	struct usb_ep_descriptor if0_in_ep;
	struct usb_desc_header nil_desc;
};

struct wusb_data {
	struct wusb_desc *const desc;
	const struct usb_desc_header **const fs_desc;
};

static struct usbd_class_data *wusb_cd;

static uint8_t out_ep(struct usbd_class_data *c)
{
	struct wusb_data *d = (struct wusb_data *)usbd_class_get_private(c);
	return d->desc->if0_out_ep.bEndpointAddress;
}
static uint8_t in_ep(struct usbd_class_data *c)
{
	struct wusb_data *d = (struct wusb_data *)usbd_class_get_private(c);
	return d->desc->if0_in_ep.bEndpointAddress;
}

static struct net_buf *alloc_buf(uint8_t ep)
{
	struct net_buf *buf = net_buf_alloc(&wusb_pool, K_NO_WAIT);
	if (!buf)
		return NULL;
	struct udc_buf_info *bi = udc_get_buf_info(buf);
	memset(bi, 0, sizeof(*bi));
	bi->ep = ep;
	return buf;
}

static void queue_out(struct usbd_class_data *c)
{
	struct net_buf *buf = alloc_buf(out_ep(c));
	if (!buf)
		return;
	if (usbd_ep_enqueue(c, buf))
		net_buf_unref(buf);
}

static int wusb_request(struct usbd_class_data *c, struct net_buf *buf, int err)
{
	struct usbd_context *ctx = usbd_class_get_ctx(c);
	struct udc_buf_info *bi = (struct udc_buf_info *)net_buf_user_data(buf);

	if (err == 0 && bi->ep == out_ep(c)) {
		// browser -> device: stash payload, re-arm the OUT read
		if (buf->len)
			ring_buf_put(&wusb_rx_ring, buf->data,
				     MIN(buf->len,
					 ring_buf_space_get(&wusb_rx_ring)));
		usbd_ep_buf_free(ctx, buf);
		if (atomic_test_bit(&wusb_state, WUSB_ENABLED))
			queue_out(c);
		return 0;
	}
	usbd_ep_buf_free(ctx, buf); // IN done or error
	return 0;
}

static void *wusb_get_desc(struct usbd_class_data *const c,
			   const enum usbd_speed speed)
{
	struct wusb_data *d = (struct wusb_data *)usbd_class_get_private(c);
	(void)speed;
	return d->fs_desc;
}

static void wusb_enable(struct usbd_class_data *const c)
{
	wusb_cd = c;
	if (!atomic_test_and_set_bit(&wusb_state, WUSB_ENABLED))
		queue_out(c);
}
static void wusb_disable(struct usbd_class_data *const c)
{
	(void)c;
	atomic_clear_bit(&wusb_state, WUSB_ENABLED);
}
static int wusb_init(struct usbd_class_data *c)
{
	wusb_cd = c;
	return 0;
}

static struct usbd_class_api wusb_api = {
	.request = wusb_request,
	.enable = wusb_enable,
	.disable = wusb_disable,
	.init = wusb_init,
	.get_desc = wusb_get_desc,
};

static struct wusb_desc wusb_desc_0 = {
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_BCC_VENDOR,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,
	},
	.if0_out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = 0x02,
		.bmAttributes = USB_EP_TYPE_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(64U),
		.bInterval = 0x00,
	},
	.if0_in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = 0x82,
		.bmAttributes = USB_EP_TYPE_BULK,
		.wMaxPacketSize = sys_cpu_to_le16(64U),
		.bInterval = 0x00,
	},
	.nil_desc = {
		.bLength = 0,
		.bDescriptorType = 0,
	},
};

const static struct usb_desc_header *wusb_fs_desc_0[] = {
	(struct usb_desc_header *)&wusb_desc_0.if0,
	(struct usb_desc_header *)&wusb_desc_0.if0_in_ep,
	(struct usb_desc_header *)&wusb_desc_0.if0_out_ep,
	(struct usb_desc_header *)&wusb_desc_0.nil_desc,
};

static struct wusb_data wusb_data_0 = {
	.desc = &wusb_desc_0,
	.fs_desc = wusb_fs_desc_0,
};

USBD_DEFINE_CLASS(opk_webusb_0, &wusb_api, &wusb_data_0, NULL);

// ---- bridge to the Adafruit_USBD_WebUSB shim ----
extern "C" bool opk_webusb_connected(void)
{
	return atomic_test_bit(&wusb_state, WUSB_ENABLED);
}
extern "C" int opk_webusb_available(void)
{
	return (int)ring_buf_size_get(&wusb_rx_ring);
}
extern "C" int opk_webusb_read(void)
{
	uint8_t b;
	return ring_buf_get(&wusb_rx_ring, &b, 1) == 1 ? b : -1;
}
extern "C" uint32_t opk_webusb_write(const void *data, uint32_t n)
{
	if (!wusb_cd || !opk_webusb_connected())
		return 0;
	// Chunk into 64-byte IN transfers.
	const uint8_t *p = (const uint8_t *)data;
	uint32_t sent = 0;
	while (sent < n) {
		uint32_t chunk = MIN(n - sent, 64U);
		struct net_buf *buf = alloc_buf(in_ep(wusb_cd));
		if (!buf)
			break;
		net_buf_add_mem(buf, p + sent, chunk);
		if (usbd_ep_enqueue(wusb_cd, buf)) {
			net_buf_unref(buf);
			break;
		}
		sent += chunk;
	}
	return sent;
}
extern "C" void opk_webusb_flush(void)
{
	// writes are enqueued immediately; nothing to flush
}
