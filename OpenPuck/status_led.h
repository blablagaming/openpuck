// status_led.h -- two-LED status + wake indicator (PWM-dimmed).
//
//   Awake (bus not suspended):        BLUE on, dim                         RED off
//   Asleep / wake-armed (suspended):  BLUE off                            RED dim blink every ~2s
//   Asleep + a wake is being sent:    BLUE flashes bright (~500ms)        RED keeps blinking
//
// So at a glance: steady dim blue = running; slow dim red heartbeat = host asleep & we're armed to wake it;
// blue flash = we just fired a wake (if the PC then stays asleep, the host ignored our resume -- see
// "Wake from sleep" in ARCHITECTURE.md).
//
// Pin/level config: the sketch builds with the Feather variant but the hardware is a "Pro Micro" nRF52840
// clone, so these are the board's actual user-LED GPIOs, not the Feather's. Defaults are a best guess for the
// SuperMini clone (blue P0.15 = D24, red P1.15 = D3, active-high). If the two colors are swapped, swap the
// pins; if a color never lights, it's on a different GPIO -- override here.
#pragma once

#ifndef LED_BLUE_PIN
#define LED_BLUE_PIN 24            // P0.15 (D24 in the Feather pin map)
#endif
#ifndef LED_RED_PIN
#define LED_RED_PIN  LED_BUILTIN   // P1.15 (D3)
#endif
#ifndef LED_ACTIVE_HIGH
#define LED_ACTIVE_HIGH 1          // 1 = LED lit when pin driven high; set 0 for active-low boards
#endif

// brightness 0..255 (PWM duty)
#ifndef LED_BLUE_DIM
#define LED_BLUE_DIM   16          // "on, dim" steady blue during normal operation
#endif
#ifndef LED_BLUE_FLASH
#define LED_BLUE_FLASH 255         // bright blue flash when a wake is sent
#endif
#ifndef LED_RED_DIM
#define LED_RED_DIM    16          // dim red heartbeat while wake-armed
#endif

void ledInit();        // call once from setup()
void ledWakePulse();   // call at each USBDevice.remoteWakeup() site -> blue bright for ~500ms
void ledTask();        // call every loop()
