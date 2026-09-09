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

  // RTOS experiment entry points.  Ownership is deliberately explicit:
  // rtosSensorTick() is the only task that may transact on Wire, while the
  // control task is the only task that may access the Dynamixel UART.
  void rtosSensorTick(uint32_t nowMs);
  void rtosControlTick(uint32_t nowMs);
  void rtosLoggerTick(uint32_t nowMs);
  void setRtosStackFreeWords(uint16_t controlWords, uint16_t sensorWords,
                             uint16_t loggerWords);

 private:
  Stream& debug_;
  Bno085Sensor imu_;
  Bmp581Sensor barometer_;
  MaxM10sSensor gps_;
  E22Receiver e22_;
  GimbalController gimbal_;
  SdLogger logger_;

  TrackMode mode_ = TrackMode::BOOT;
  TrackingBlocker blocker_ = TrackingBlocker::BARO_BIAS_NOT_READY;
  RelativeTarget relative_;
  // Last target in the IMU/alignment reference frame. It enables immediate
  // IMU-only stabilization and later preserves LOS during data loss.
  float lastTargetNed_[3] = {1.0f, 0.0f, 0.0f};
  bool haveLastTargetNed_ = false;
  bool imuHoldReferenceReady_ = false;
  bool baroBiasCaptureActive_ = false;
  bool baroBiasReady_ = false;
  float initialBaroBiasM_ = 0.0f;
  float baroBiasSumM_ = 0.0f;
  uint16_t baroBiasSamples_ = 0;

  // Averaged while the user holds the gimbal aimed at the transmitter during
  // an explicit initial alignment window.
  float localImuReferenceQ_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float initialImuSum_[4] = {};
  float initialTargetDirectionSum_[3] = {};
  float initialTargetYawDeg_ = 0.0f;
  float initialTargetPitchDeg_ = 0.0f;
  uint32_t initialAlignmentStartMs_ = 0;
  uint16_t initialAlignmentSamples_ = 0;
  bool trackingReferenceReady_ = false;
  bool haveAlignmentSequence_ = false;
  uint8_t lastAlignmentSequence_ = 0;
  bool haveBaroBiasSequence_ = false;
  uint8_t lastBaroBiasSequence_ = 0;

  uint32_t lastControlUs_ = 0;
  uint32_t lastGpsPollUs_ = 0;
  uint32_t lastBarometerPollMs_ = 0;
  uint32_t lastSdLogMs_ = 0;
  uint32_t lastStatusMs_ = 0;
  // Minimum unused FreeRTOS task-stack depth observed, in 32-bit words.
  uint16_t controlStackFreeWords_ = 0;
  uint16_t sensorStackFreeWords_ = 0;
  uint16_t loggerStackFreeWords_ = 0;
  uint8_t backgroundSlot_ = 0;
  bool nextTemperatureIsYaw_ = true;
  // The RTOS SD diagnostic includes stage-level timing fields.  512 bytes is
  // insufficient when every counter reaches ten digits; this test-only
  // buffer must retain the complete line rather than silently drop it.
  char statusTxBuffer_[768] = {};
  size_t statusTxLength_ = 0;
  size_t statusTxOffset_ = 0;
  // Zero means there is no uninterrupted healthy input window yet.
  uint32_t trackingRecoveryStartMs_ = 0;

  void controlTick(uint32_t nowMs, float dtSeconds);
  void backgroundTick(uint32_t nowMs);
  void rtosFeedbackTick();
  void sdLogTick(uint32_t nowMs);
  void queueStatus(uint32_t nowMs);
  void serviceStatusTx();
  bool trackingInputsFresh(uint32_t nowMs) const;
  bool attitudeFresh(uint32_t nowMs) const;
  bool localBarometerFresh(uint32_t nowMs) const;
  bool alignmentInputsFresh(uint32_t nowMs) const;
  TrackingBlocker inputBlocker(uint32_t nowMs) const;
  bool captureBaroBiasSample(uint32_t nowMs);
  bool targetGeometry(float ned[3], RelativeTarget* calculated) const;
  bool initializeImuHold(uint32_t nowMs);
  bool commandHeldDirection(float dtSeconds);
  void enterSafeHold(TrackingBlocker blocker, bool disableTracking);
  void resetInitialAlignment(uint32_t nowMs);
  void collectInitialAlignmentSample(uint32_t nowMs);
  bool finishInitialAlignment(uint32_t nowMs);
  static const char* modeName(TrackMode mode);
  static const char* blockerName(TrackingBlocker blocker);
  const GpsFix& localGpsInput() const;
  const BarometerSample& localBarometerInput() const;
  const RemoteTargetSample& remoteInput() const;
};
