#include "status_led.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

#define WAKE_LED_OFF ((WAKE_LED_ON)==HIGH ? LOW : HIGH)
#define BLIP_PERIOD_MS 2000u   // one blip every 2s while wake-armed
#define BLIP_ON_MS       60u   // ~3% duty: visible at a glance, not a bedroom nightlight

static void ledWrite(int level){
  digitalWrite(WAKE_LED_PIN_A, level);
  digitalWrite(WAKE_LED_PIN_B, level);
}

void ledInit(){
  pinMode(WAKE_LED_PIN_A, OUTPUT);
  pinMode(WAKE_LED_PIN_B, OUTPUT);
  ledWrite(WAKE_LED_OFF);
}

void ledTask(){
  static bool lit=false;
  bool want=false;
  if (USBDevice.suspended()){                       // wake-armed: blip BLIP_ON_MS out of every BLIP_PERIOD_MS
    want = (millis() % BLIP_PERIOD_MS) < BLIP_ON_MS;
  }
  if (want != lit){ lit = want; ledWrite(lit ? WAKE_LED_ON : WAKE_LED_OFF); }   // write only on change
}
