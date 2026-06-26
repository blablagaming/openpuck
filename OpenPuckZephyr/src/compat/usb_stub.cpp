// usb_stub.cpp -- inert TinyUSB-shim backing used during core bring-up.
//
// Lets the non-USB firmware (radio/protocol/modes-logic/data) compile and link
// before the real Zephyr USBD backing (usb_glue.cpp + DT HID nodes + usbd
// context) is wired in. Every method is a safe no-op: not mounted/suspended,
// HIDs never ready so sendReport()s are skipped. Swapped out for usb_glue.cpp
// once the USBD stack is up.
#include "Adafruit_TinyUSB.h"
#include "usb_identity.h"
#include <string.h>

opk_usb_identity g_opk_usb_id;

Adafruit_USBD_HID::Adafruit_USBD_HID()
{
}
void Adafruit_USBD_HID::setReportDescriptor(const uint8_t *desc, uint16_t len)
{
	_desc = desc;
	_desc_len = len;
}
void Adafruit_USBD_HID::setReportCallback(hid_get_report_cb_t g,
					  hid_set_report_cb_t s)
{
	_get_cb = g;
	_set_cb = s;
}
void Adafruit_USBD_HID::setStringDescriptor(const char *s)
{
	_str = s;
}
void Adafruit_USBD_HID::setPollInterval(uint8_t i)
{
	_poll_ms = i;
}
void Adafruit_USBD_HID::setBootProtocol(uint8_t p)
{
	_boot_proto = p;
}
void Adafruit_USBD_HID::enableOutEndpoint(bool e)
{
	_has_out = e;
}
bool Adafruit_USBD_HID::begin()
{
	return true;
}
bool Adafruit_USBD_HID::ready()
{
	return false;
}
bool Adafruit_USBD_HID::sendReport(uint8_t, const void *, uint16_t)
{
	return false;
}

Adafruit_USBD_WebUSB::Adafruit_USBD_WebUSB()
{
}
bool Adafruit_USBD_WebUSB::begin()
{
	return true;
}
bool Adafruit_USBD_WebUSB::connected()
{
	return false;
}
int Adafruit_USBD_WebUSB::available()
{
	return 0;
}
int Adafruit_USBD_WebUSB::read()
{
	return -1;
}
uint32_t Adafruit_USBD_WebUSB::write(const void *, uint32_t n)
{
	return n;
}
void Adafruit_USBD_WebUSB::flush()
{
}

void Adafruit_USBD_Device::setID(uint16_t v, uint16_t p)
{
	g_opk_usb_id.vid = v;
	g_opk_usb_id.pid = p;
}
void Adafruit_USBD_Device::setVersion(uint16_t b)
{
	g_opk_usb_id.bcd_usb = b;
}
void Adafruit_USBD_Device::setDeviceVersion(uint16_t b)
{
	g_opk_usb_id.bcd_device = b;
}
void Adafruit_USBD_Device::setManufacturerDescriptor(const char *s)
{
	g_opk_usb_id.manufacturer = s;
}
void Adafruit_USBD_Device::setProductDescriptor(const char *s)
{
	g_opk_usb_id.product = s;
}
void Adafruit_USBD_Device::setSerialDescriptor(const char *s)
{
	g_opk_usb_id.serial = s;
}
void Adafruit_USBD_Device::setConfigurationBuffer(uint8_t *, uint32_t)
{
}
void Adafruit_USBD_Device::setConfigurationAttribute(uint8_t a)
{
	g_opk_usb_id.cfg_attr = a;
}
uint8_t Adafruit_USBD_Device::allocInterface(uint8_t count)
{
	uint8_t b = g_opk_usb_id.next_itf;
	g_opk_usb_id.next_itf += count;
	return b;
}
uint8_t Adafruit_USBD_Device::allocEndpoint(uint8_t dir)
{
	uint8_t n = ++g_opk_usb_id.next_ep;
	return (dir ? 0x80 : 0x00) | n;
}
bool Adafruit_USBD_Device::addInterface(Adafruit_USBD_Interface &)
{
	return true;
}
void Adafruit_USBD_Device::clearConfiguration()
{
	g_opk_usb_id.next_itf = 0;
	g_opk_usb_id.next_ep = 0;
}
bool Adafruit_USBD_Device::detach()
{
	return true;
}
bool Adafruit_USBD_Device::attach()
{
	return true;
}
bool Adafruit_USBD_Device::mounted()
{
	return true;
}
bool Adafruit_USBD_Device::suspended()
{
	return false;
}
bool Adafruit_USBD_Device::remoteWakeup()
{
	return true;
}
Adafruit_USBD_Device USBDevice;
Adafruit_USBD_Device &TinyUSBDevice = USBDevice;

// DFU entry: stash the Adafruit-bootloader magic in GPREGRET and reset. 0x57 ==
// UF2 mass-storage; 0x4E == serial (OTA) DFU. (Inert here only in that the host
// reconnects to the bootloader; functional once flashed.)
#include "Arduino.h"
void enterUf2Dfu(void)
{
	NRF_POWER->GPREGRET = 0x57;
	NVIC_SystemReset();
}
void enterSerialDfu(void)
{
	NRF_POWER->GPREGRET = 0x4E;
	NVIC_SystemReset();
}

// ---- TinyUSB custom-class-driver stubs (XInput) ----
// The XInput interface is a custom TinyUSB class driver; these are the endpoint
// primitives it calls. Stubbed during bring-up (XInput will not enumerate until
// the Zephyr custom class is implemented — see mode_xinput port).
#include "device/usbd_pvt.h"
extern "C" bool tud_mounted(void)
{
	return USBDevice.mounted();
}
bool usbd_edpt_open(uint8_t rhport, const tusb_desc_endpoint_t *ep)
{
	(void)rhport;
	(void)ep;
	return false;
}
bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep, uint8_t *buf, uint16_t n)
{
	(void)rhport;
	(void)ep;
	(void)buf;
	(void)n;
	return false;
}
bool usbd_edpt_busy(uint8_t rhport, uint8_t ep)
{
	(void)rhport;
	(void)ep;
	return false;
}
bool usbd_edpt_claim(uint8_t rhport, uint8_t ep)
{
	(void)rhport;
	(void)ep;
	return false;
}
void usbd_edpt_release(uint8_t rhport, uint8_t ep)
{
	(void)rhport;
	(void)ep;
}
