// Arduino.h -- compatibility shim for the Zephyr port.
//
// The OpenPuck modules were written against the Arduino/Adafruit-nRF52 core.
// Rather than rewrite the reverse-engineered radio/protocol/mode logic, this
// header provides the small Arduino API surface they actually use (timing,
// GPIO, Serial, math helpers) on top of Zephyr, plus the Nordic MDK register
// definitions (NRF_RADIO/NRF_WDT/NRF_FICR/NRF_CLOCK) and CMSIS intrinsics that
// the bare-metal radio code pokes directly. Those register macros come from the
// same Nordic MDK that Zephyr's hal_nordic ships, so the radio code ports
// essentially verbatim.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Nordic MDK: NRF_RADIO, NRF_WDT, NRF_FICR, NRF_CLOCK, the RADIO_*/WDT_* bitfield
// macros, and the CMSIS-Core intrinsics (__disable_irq/__enable_irq,
// NVIC_SystemReset, DWT). nrfx.h pulls in the MDK device + bitfield headers and
// the CMSIS core for the nRF52840, and is on hal_nordic's include path.
#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- timing ----
// millis(): kernel uptime in ms. micros(): a free-running 1 MHz-equivalent
// counter derived from the Cortex-M DWT cycle counter (see arduino_compat.cpp).
// Both are monotonic 32-bit and meant for unsigned-delta use (wrap is fine).
uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

// ---- GPIO (status LED only; resolved against devicetree in arduino_compat) ----
// LED_BUILTIN: the Feather user LED P1.15, as an absolute nRF pin (port 1 * 32 +
// 15 = 47), matching the absolute-pin convention of this shim's digitalWrite.
#define LED_BUILTIN 47
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
void pinMode(uint32_t pin, uint32_t mode);
void digitalWrite(uint32_t pin, uint32_t val);
int digitalRead(uint32_t pin);

// ---- misc ----
long random(long howbig);
long random_range(long howsmall, long howbig);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// Arduino map(): integer rescale, same semantics as the AVR core.
static inline long map(long x, long in_min, long in_max, long out_min,
		       long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename T> static inline T mn(T a, T b)
{
	return a < b ? a : b;
}
template <typename T> static inline T mx(T a, T b)
{
	return a > b ? a : b;
}
#ifndef min
#define min(a, b) mn(a, b)
#endif
#ifndef max
#define max(a, b) mx(a, b)
#endif
#ifndef constrain
#define constrain(amt, low, high) \
	((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// Arduino's Serial: the modules call printf/print/println/availableForWrite/
// available/read. Backed by the USB CDC console once up, by the Zephyr console
// (RTT/UART) during bring-up. See arduino_compat.cpp.
class SerialShim {
    public:
	void begin(unsigned long baud)
	{
		(void)baud;
	}
	int printf(const char *fmt, ...);
	void print(const char *s);
	void print(char c);
	void println(const char *s);
	void println(void);
	int available(void);
	int availableForWrite(void);
	int read(void);
	size_t write(uint8_t b);
	explicit operator bool() const
	{
		return true;
	}
};
extern SerialShim Serial;
#endif
