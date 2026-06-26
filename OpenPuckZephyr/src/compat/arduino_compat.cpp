// arduino_compat.cpp -- implementation of the Arduino API shim (see Arduino.h).
#include "Arduino.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/gpio.h>

// ---- microsecond clock via the Cortex-M DWT cycle counter ----
// The RF poll busy-waits ((micros() - t0) < window_us) need ~us resolution; the
// kernel RTC tick (32.768 kHz) is far too coarse. DWT->CYCCNT counts CPU cycles
// at SystemCoreClock (64 MHz on the nRF52840), so micros = CYCCNT / 64. The
// 32-bit counter wraps every ~67 s, which is harmless under the unsigned-delta
// comparisons the callers use.
static inline void dwt_init(void)
{
	static bool done;
	if (done)
		return;
	done = true;
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

extern "C" uint32_t micros(void)
{
	dwt_init();
	return DWT->CYCCNT / (SystemCoreClock / 1000000UL);
}

extern "C" uint32_t millis(void)
{
	return k_uptime_get_32();
}

extern "C" void delay(uint32_t ms)
{
	k_msleep((int32_t)ms);
}

extern "C" void delayMicroseconds(uint32_t us)
{
	k_busy_wait(us);
}

// ---- GPIO ----
// The Arduino "pin number" here is the absolute nRF pin index (port * 32 + pin),
// which is what the ported status_led code passes. We drive it through nrfx via
// the SoC GPIO registers so no devicetree node is required for an arbitrary pin.
#include <hal/nrf_gpio.h>

extern "C" void pinMode(uint32_t pin, uint32_t mode)
{
	if (mode == OUTPUT)
		nrf_gpio_cfg_output(pin);
	else
		nrf_gpio_cfg_input(pin, mode == INPUT_PULLUP ?
						NRF_GPIO_PIN_PULLUP :
						NRF_GPIO_PIN_NOPULL);
}

extern "C" void digitalWrite(uint32_t pin, uint32_t val)
{
	if (val)
		nrf_gpio_pin_set(pin);
	else
		nrf_gpio_pin_clear(pin);
}

extern "C" int digitalRead(uint32_t pin)
{
	return (int)nrf_gpio_pin_read(pin);
}

// ---- misc ----
extern "C" long random(long howbig)
{
	if (howbig <= 0)
		return 0;
	return (long)(sys_rand32_get() % (uint32_t)howbig);
}

extern "C" long random_range(long howsmall, long howbig)
{
	if (howbig <= howsmall)
		return howsmall;
	return howsmall + random(howbig - howsmall);
}

// ---- Serial ----
// During bring-up Serial output goes to the Zephyr console (printk -> RTT/UART).
// Once the USB CDC console is up, serial_console's I/O is wired through it; the
// input path (available/read) is overridden there. printk is used for output so
// early boot logging works before USB enumerates.
SerialShim Serial;

extern "C" __attribute__((weak)) size_t opk_serial_write(uint8_t b);

static void serial_puts(const char *s, size_t n)
{
	for (size_t i = 0; i < n; i++)
		opk_serial_write((uint8_t)s[i]);
}

int SerialShim::printf(const char *fmt, ...)
{
	char buf[256];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof buf, fmt, ap);
	va_end(ap);
	serial_puts(buf, (n > 0 && n < (int)sizeof buf) ? (size_t)n : strlen(buf));
	return n;
}

void SerialShim::print(const char *s)
{
	serial_puts(s, strlen(s));
}

void SerialShim::print(char c)
{
	opk_serial_write((uint8_t)c);
}

void SerialShim::println(const char *s)
{
	serial_puts(s, strlen(s));
	opk_serial_write('\n');
}

void SerialShim::println(void)
{
	opk_serial_write('\n');
}

// Input/flow-control backend: weak free functions so the USB CDC layer can
// strongly override them once it is up. During bring-up they stub out (no input,
// always writable, output to the console).
extern "C" __attribute__((weak)) int opk_serial_available(void)
{
	return 0;
}
extern "C" __attribute__((weak)) int opk_serial_avail_write(void)
{
	return 64;
}
extern "C" __attribute__((weak)) int opk_serial_read(void)
{
	return -1;
}
extern "C" __attribute__((weak)) size_t opk_serial_write(uint8_t b)
{
	printk("%c", b);
	return 1;
}

int SerialShim::available(void)
{
	return opk_serial_available();
}
int SerialShim::availableForWrite(void)
{
	return opk_serial_avail_write();
}
int SerialShim::read(void)
{
	return opk_serial_read();
}
size_t SerialShim::write(uint8_t b)
{
	return opk_serial_write(b);
}
