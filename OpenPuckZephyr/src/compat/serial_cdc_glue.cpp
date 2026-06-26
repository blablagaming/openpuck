// serial_cdc_glue.cpp -- routes the Arduino Serial shim to the USB CDC ACM.
//
// The board's chosen console is the CDC ACM UART (board_cdc_acm_uart), so
// Serial output already reaches USB via printk. This unit additionally wires the
// input side (serial_console's single-letter command reader) and a direct write
// path by strongly overriding the weak opk_serial_* hooks in arduino_compat.cpp.
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <stdint.h>
#include <stddef.h>

static const struct device *const cdc = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

// One-byte pushback so available()+read() work without a UART peek primitive.
static int s_have = -1;

extern "C" int opk_serial_available(void)
{
	if (s_have >= 0)
		return 1;
	unsigned char c;
	if (device_is_ready(cdc) && uart_poll_in(cdc, &c) == 0) {
		s_have = c;
		return 1;
	}
	return 0;
}

extern "C" int opk_serial_read(void)
{
	if (s_have >= 0) {
		int c = s_have;
		s_have = -1;
		return c;
	}
	unsigned char c;
	if (device_is_ready(cdc) && uart_poll_in(cdc, &c) == 0)
		return c;
	return -1;
}

extern "C" int opk_serial_avail_write(void)
{
	// The firmware gates its diagnostic prints on availableForWrite() > 80/90.
	// Report a generous CDC TX budget so those logs are not suppressed (the
	// underlying uart_poll_out drops bytes when the host isn't draining, so
	// over-reporting can't wedge the loop).
	return 256;
}

extern "C" size_t opk_serial_write(uint8_t b)
{
	if (device_is_ready(cdc))
		uart_poll_out(cdc, b);
	return 1;
}
