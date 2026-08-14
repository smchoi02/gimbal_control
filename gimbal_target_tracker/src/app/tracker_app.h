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
  // Averaged while the user holds the gimbal aimed at the transmitter during
  // the initial alignment window.
  float localImuReferenceQ_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float initialImuSum_[4] = {};
  float initialTargetDirectionSum_[3] = {};
  float initialTargetYawDeg_ = 0.0f;
  float initialTargetPitchDeg_ = 0.0f;
  uint32_t initialAlignmentStartMs_ = 0;
  uint16_t initialAlignmentSamples_ = 0;
  bool trackingReferenceReady_ = false;

  uint32_t lastControlUs_ = 0;
  uint32_t lastBaroUs_ = 0;
  uint32_t lastGpsPollUs_ = 0;
  uint32_t lastLogUs_ = 0;
  uint32_t lastStatusMs_ = 0;

  char commandBuffer_[48] = {};
  uint8_t commandLength_ = 0;

  void controlTick(uint32_t nowMs, float dtSeconds);
  void logTick(uint32_t nowMs);
  void printStatus(uint32_t nowMs);
  void pollCommands();
  void handleCommand(char* line);
  bool trackingInputsFresh(uint32_t nowMs) const;
  bool attitudeFresh(uint32_t nowMs) const;
  void resetInitialAlignment(uint32_t nowMs);
  void collectInitialAlignmentSample(uint32_t nowMs);
  bool finishInitialAlignment(uint32_t nowMs);
  const GpsFix& localGpsInput() const;
  const BarometerSample& localBarometerInput() const;
  const RemoteTargetSample& remoteInput() const;
};
