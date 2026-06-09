// status_led.h -- LED indication of the wake-armed state.
//
// When the host puts the bus in suspend (PC asleep) the device is "wake-armed": input forwarding is gated off
// and only the explicit gestures (Steam-button short press / controller connect) will wake the host (see
// "Wake from sleep" in ARCHITECTURE.md). That state was previously invisible -- you couldn't tell whether the
// puck considered the host asleep. ledTask() makes it visible: a short LED blip every ~2s while wake is armed,
// LED off otherwise (normal operation keeps the LED dark; the blip duty is ~3% so it won't light up a bedroom).
//
// Board note: the sketch is built with the Feather nRF52840 variant, but the usual hardware is a SuperMini
// "Pro Micro" clone. The Feather's user LED is P1.15 (D3, active high); the SuperMini's blue user LED is
// P0.15 (= D24 in the Feather pin map -- SPI MISO, unused by this project). We drive BOTH pins so the
// indicator works on either board. Override the pins/polarity below if your board differs.
#pragma once

#ifndef WAKE_LED_PIN_A
#define WAKE_LED_PIN_A LED_BUILTIN   // Feather: P1.15 user LED (harmless unconnected pad on SuperMini clones)
#endif
#ifndef WAKE_LED_PIN_B
#define WAKE_LED_PIN_B 24            // SuperMini "Pro Micro" clone: P0.15 blue user LED (D24 in the Feather map)
#endif
#ifndef WAKE_LED_ON
#define WAKE_LED_ON HIGH             // set LOW if your board's LED is wired active-low (blip pattern inverts)
#endif

void ledInit();   // call once from setup(): pins to output, LED off
void ledTask();   // call every loop(): blip while USBDevice.suspended(), dark otherwise
