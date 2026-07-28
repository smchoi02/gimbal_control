#include <Arduino.h>
#include <Wire.h>

#include "src/app/tracker_app.h"
#include "src/config/system_config.h"

#define DEBUG_SERIAL Serial
#define LORA_SERIAL Serial3

TrackerApp app(DEBUG_SERIAL, LORA_SERIAL);

void setup() {
  DEBUG_SERIAL.begin(115200);
  const uint32_t waitStart = millis();
  while (!DEBUG_SERIAL && millis() - waitStart < 3000) {}

  Wire.begin();
  Wire.setClock(cfg::I2C_CLOCK_HZ);
  LORA_SERIAL.begin(cfg::LORA_UART_BAUD);
  app.begin();
}

void loop() {
  app.update();
}
