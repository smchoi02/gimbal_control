#include <Arduino.h>
#include <Wire.h>
#include <FreeRTOS_SAMD21.h>

#include "src/app/tracker_app.h"
#include "src/config/system_config.h"

#define DEBUG_SERIAL Serial
#define E22_SERIAL Serial3

TrackerApp app(DEBUG_SERIAL, E22_SERIAL);

namespace {
constexpr uint16_t CONTROL_STACK_WORDS = 384;
constexpr uint16_t SENSOR_STACK_WORDS = 512;
constexpr uint16_t LOGGER_STACK_WORDS = 512;
constexpr UBaseType_t CONTROL_TASK_PRIORITY = tskIDLE_PRIORITY + 3;
// Sensor and logger are peers.  Their operations are independent, and a
// higher sensor priority previously stretched every SD operation into
// hundreds of milliseconds by repeatedly starving loggerTask.
constexpr UBaseType_t BACKGROUND_TASK_PRIORITY = tskIDLE_PRIORITY + 1;

TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t loggerTaskHandle = nullptr;
uint32_t lastStackCheckMs = 0;

void controlTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    app.rtosControlTick(millis());
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
  }
}

void sensorTask(void*) {
  for (;;) {
    if (!cfg::RTOS_DISABLE_SENSOR_IO_SD_ISOLATION_TEST) {
      app.rtosSensorTick(millis());
    }
    const uint32_t nowMs = millis();
    if (static_cast<uint32_t>(nowMs - lastStackCheckMs) >= 1000U) {
      lastStackCheckMs = nowMs;
      // High-water mark is the minimum free depth observed since creation.
      // StackType_t is 32 bits on SAMD21, so these values are 4-byte words.
      app.setRtosStackFreeWords(
          static_cast<uint16_t>(uxTaskGetStackHighWaterMark(controlTaskHandle)),
          static_cast<uint16_t>(uxTaskGetStackHighWaterMark(sensorTaskHandle)),
          static_cast<uint16_t>(uxTaskGetStackHighWaterMark(loggerTaskHandle)));
    }
    // Yield promptly.  This task owns I2C, but is lower priority than control.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void loggerTask(void*) {
  for (;;) {
    // SensorTask remains runnable during SD I/O. Suspending it while a card
    // waits for a busy timeout turns a storage fault into an IMU-stale fault.
    app.rtosLoggerTick(millis());
    // The 10 Hz decision is inside rtosLoggerTick; this only prevents a busy
    // loop between samples and leaves all spare CPU to control/sensors.
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
}  // namespace

void setup() {
  DEBUG_SERIAL.begin(115200);
  const uint32_t waitStart = millis();
  while (!DEBUG_SERIAL && millis() - waitStart < 3000) {}

  Wire.begin();
  Wire.setClock(cfg::I2C_CLOCK_HZ);
  E22_SERIAL.begin(cfg::E22_UART_BAUD);
  app.begin();

  // No I2C/UART transaction is performed from an ISR.  The 1 kHz RTOS tick
  // releases the 50 Hz control task, which preempts loggerTask during SD busy
  // polling as long as the driver keeps global interrupts enabled.
  const BaseType_t controlCreated = xTaskCreate(
      controlTask, "control", CONTROL_STACK_WORDS, nullptr,
      CONTROL_TASK_PRIORITY, &controlTaskHandle);
  const BaseType_t sensorCreated = xTaskCreate(
      sensorTask, "sensor", SENSOR_STACK_WORDS, nullptr,
      BACKGROUND_TASK_PRIORITY, &sensorTaskHandle);
  const BaseType_t loggerCreated = xTaskCreate(
      loggerTask, "logger", LOGGER_STACK_WORDS, nullptr,
      BACKGROUND_TASK_PRIORITY, &loggerTaskHandle);
  if (controlCreated != pdPASS || sensorCreated != pdPASS ||
      loggerCreated != pdPASS) {
    DEBUG_SERIAL.println(F("rtos: task allocation failed"));
    for (;;) {}
  }
  DEBUG_SERIAL.println(F("rtos: scheduler start"));
  vTaskStartScheduler();
  DEBUG_SERIAL.println(F("rtos: scheduler failed"));
}

void loop() {
  // FreeRTOS owns the MCU after setup().
}
