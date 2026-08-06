#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../actuators/gimbal_controller.h"
#include "../comm/e22_receiver.h"
#include "../common/types.h"
#include "../sensors/bmp581_sensor.h"
#include "../sensors/bno085_sensor.h"
#include "../sensors/max_m10s_sensor.h"
#include "../storage/sd_logger.h"

class TrackerApp {
 public:
  TrackerApp(Stream& debug, Stream& lora);

  void begin();
  void update();

 private:
  Stream& debug_;
  Bno085Sensor imu_;
  Bmp581Sensor barometer_;
  MaxM10sSensor gps_;
  E22Receiver e22_;
  GimbalController gimbal_;
  SdLogger logger_;

  TrackMode mode_ = TrackMode::STOW;
  RelativeTarget relative_;
  float lastDirectionNed_[3] = {1.0f, 0.0f, 0.0f};
  bool haveLastDirection_ = false;
  bool trackingEnabled_ = true;

  // Manual aiming, for bench work with no transmitter present.
  bool manualActive_ = false;
  float manualYawDeg_ = 0.0f;
  float manualPitchDeg_ = 0.0f;
  uint32_t manualCommandMs_ = 0;

  uint32_t lastControlUs_ = 0;
  uint32_t lastBaroUs_ = 0;
  uint32_t lastGpsPollUs_ = 0;
  uint32_t lastLogUs_ = 0;
  uint32_t lastStatusMs_ = 0;

  // A remote sample pushed in over USB. It takes over from the radio while it
  // keeps arriving, so the full pipeline can be exercised on a bench with the
  // real payload sensors and no transmitter built yet.
  RemoteTargetSample injected_;

  char commandBuffer_[96] = {};
  uint8_t commandLength_ = 0;

  void controlTick(uint32_t nowMs, float dtSeconds);
  // Freeze the current boresight as a world direction so the gimbal holds it
  // against payload motion. This is the stabilisation test with no target.
  bool captureHoldDirection();
  void logTick(uint32_t nowMs);
  void printStatus(uint32_t nowMs);
  void pollCommands();
  void handleCommand(char* line);
  bool trackingInputsFresh(uint32_t nowMs) const;
  bool attitudeFresh(uint32_t nowMs) const;
  const GpsFix& localGpsInput() const;
  const BarometerSample& localBarometerInput() const;
  const RemoteTargetSample& remoteInput() const;
};
