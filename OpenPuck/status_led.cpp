#include "status_led.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

#define BLINK_PERIOD_MS 2000u   // red heartbeat: one dim blip every 2s while wake-armed
#define BLINK_ON_MS       80u
#define WAKE_FLASH_MS    500u    // blue flash duration when a wake is sent

static bool          g_flash = false;
static unsigned long g_flashMs = 0;

// brightness 0 = off .. 255 = full; mapped for LED polarity
static void ledWrite(int pin, uint8_t brightness){
#if LED_ACTIVE_HIGH
  analogWrite(pin, brightness);
#else
  analogWrite(pin, (uint8_t)(255 - brightness));
#endif
}

void ledInit(){
  ledWrite(LED_BLUE_PIN, 0);
  ledWrite(LED_RED_PIN, 0);
}

void ledWakePulse(){
  g_flash = true; g_flashMs = millis();   // ledTask() picks it up on the next loop (sub-ms)
}

void ledTask(){
  static int lastBlue = -1, lastRed = -1;   // force the first write
  if (g_flash && millis()-g_flashMs >= WAKE_FLASH_MS) g_flash = false;

  uint8_t blue, red;
  if (!USBDevice.suspended()){
    blue = LED_BLUE_DIM; red = 0;                                           // awake
  } else {
    blue = g_flash ? LED_BLUE_FLASH : 0;                                    // asleep: blue only on a wake flash
    red  = ((millis() % BLINK_PERIOD_MS) < BLINK_ON_MS) ? LED_RED_DIM : 0;  // dim heartbeat
  }
  if ((int)blue != lastBlue){ lastBlue = blue; ledWrite(LED_BLUE_PIN, blue); }
  if ((int)red  != lastRed ){ lastRed  = red;  ledWrite(LED_RED_PIN,  red ); }
}
