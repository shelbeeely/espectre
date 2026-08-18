/*
 * ESPectre Core - Arduino-native orchestrator
 *
 * Thin, ESPHome-free wrapper around the ESPectre CSI motion-detection engine
 * (CSIManager, detectors, NBVI calibrator, gain controller, traffic
 * generator) for plain Arduino/PlatformIO sketches. Ported from the ESPHome
 * external_component's ESpectreComponent (see components/espectre/espectre.h
 * in the main ESPectre repo) with the Home Assistant sensor/switch/number
 * glue and BLE telemetry channel removed.
 *
 * Usage:
 *   #include <WiFi.h>
 *   #include <espectre_core.h>
 *
 *   espectre::EspectreCore motion;
 *
 *   void setup() {
 *     WiFi.mode(WIFI_STA);
 *     WiFi.begin("ssid", "password");
 *     motion.begin();  // configures CSI-critical WiFi params + starts detection
 *   }
 *
 *   void loop() {
 *     motion.loop();
 *     if (motion.isMotionDetected()) { ... }
 *   }
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * License: GPLv3
 */

#pragma once

#include <cstdint>
#include <functional>

#include "esp_err.h"

#include "utils.h"
#include "threshold.h"
#include "base_detector.h"
#include "mvs_detector.h"
#include "ml_detector.h"
#include "gain_controller.h"
#include "csi_manager.h"
#include "wifi_lifecycle.h"
#include "nbvi_calibrator.h"
#include "traffic_generator_manager.h"
#include "udp_listener.h"

namespace espectre {

// Motion state change callback, fired as soon as the detector flips state
// (before the packet-count-gated periodic callback below).
using motion_state_callback_t = std::function<void(MotionState)>;

// Periodic callback fired every `publish_interval` CSI packets, once
// detection is ready (gain locked + calibrated). Mirrors CSIManager's own
// packet callback.
using motion_update_callback_t = std::function<void(MotionState state, float movement, uint32_t packets_received)>;

/**
 * Configuration for EspectreCore::begin().
 *
 * Mirrors the YAML options of the ESPHome component; defaults match the
 * ESPHome component's defaults.
 */
struct EspectreConfig {
  DetectionAlgorithm algorithm = DetectionAlgorithm::MVS;

  // Segmentation / detection
  ThresholdMode threshold_mode = ThresholdMode::AUTO;
  float manual_threshold = 1.0f;  // only used when threshold_mode == MANUAL
  uint16_t window_size = DETECTOR_DEFAULT_WINDOW_SIZE;
  uint32_t evaluation_interval = 25;
  uint8_t motion_on_hits = 3;
  uint8_t motion_off_hits = 3;

  // Filters
  bool lowpass_enabled = false;
  float lowpass_cutoff_hz = LOWPASS_CUTOFF_DEFAULT;
  bool hampel_enabled = true;
  uint8_t hampel_window = HAMPEL_TURBULENCE_WINDOW_DEFAULT;
  float hampel_threshold = HAMPEL_TURBULENCE_THRESHOLD_DEFAULT;

  // Gain lock
  GainLockMode gain_lock_mode = GainLockMode::AUTO;

  // Subcarriers: leave null to auto-select via NBVI calibration.
  // If set, must point at 12 subcarrier indices and stays fixed (no
  // auto-calibration of the band, only of the baseline threshold).
  const uint8_t* fixed_subcarriers = nullptr;

  // Packets between periodic motion_update callbacks / rate logging.
  uint32_t publish_interval = 100;

  // Synthetic traffic to keep CSI packets flowing. Set to 0 to disable the
  // built-in generator and instead rely on an external UDP source hitting
  // udp_listen_port (e.g. another device on the LAN pinging this one).
  uint32_t traffic_generator_rate_pps = 100;
  TrafficGeneratorMode traffic_generator_mode = TrafficGeneratorMode::PING;
  uint16_t udp_listen_port = 5555;
};

/**
 * EspectreCore
 *
 * Owns the full CSI motion-detection pipeline: WiFi CSI configuration, gain
 * locking, NBVI band calibration, the MVS/ML detector, and the traffic
 * generator that keeps CSI packets flowing. No ESPHome, no Home Assistant,
 * no BLE - just begin()/loop() and a handful of accessors, so it can be
 * dropped into any Arduino sketch (including ones built on other SDKs, such
 * as FreeInk, that also target the Arduino-ESP32 framework).
 */
class EspectreCore {
 public:
  /**
   * Initialize WiFi-for-CSI, the detector, and the calibration/gain-lock
   * pipeline, and register WiFi connect/disconnect handlers.
   *
   * Call once from setup(), any time after WiFi.mode(WIFI_STA) (typically
   * right after WiFi.begin()). Detection actually starts once WiFi
   * associates and calls back through the registered IP_EVENT/WIFI_EVENT
   * handlers - begin() itself does not block on a connection.
   *
   * @return ESP_OK on success. On failure, no further calls other than
   *         begin() again are valid.
   */
  esp_err_t begin(const EspectreConfig& config = EspectreConfig());

  /**
   * Pump periodic work. Call every loop() iteration.
   *
   * Currently only drains the UDP listener socket when running in external
   * traffic mode (traffic_generator_rate_pps == 0); CSI processing itself
   * happens in the WiFi driver's own callback context.
   */
  void loop();

  // ---- Motion state ----------------------------------------------------

  bool isMotionDetected() const { return motion_state_ == MotionState::MOTION; }
  MotionState getMotionState() const { return motion_state_; }

  // Current detector metric (moving variance for MVS, probability*scale for ML).
  float getMovementLevel() const { return detector_ != nullptr ? detector_->get_motion_metric() : 0.0f; }

  // True once gain is locked, calibration has completed, and packets are
  // being evaluated (i.e. isMotionDetected()/getMovementLevel() are meaningful).
  bool isReady() const { return ready_; }

  bool isGainLocked() const { return csi_manager_.is_gain_locked(); }
  bool isCalibrating() const { return nbvi_calibrator_.is_calibrating(); }

  // ---- Runtime control ---------------------------------------------------

  // Update the detection threshold at runtime (session-only; recalculated on
  // next boot/calibration unless threshold_mode == MANUAL).
  void setThreshold(float threshold);
  float getThreshold() const { return threshold_; }

  // Re-run NBVI band calibration + baseline threshold (no-op while already
  // calibrating, or before gain lock completes).
  void triggerCalibration();

  // ---- Callbacks ----------------------------------------------------------

  // Fired immediately on every detector state transition (IDLE<->MOTION).
  void onMotionStateChanged(motion_state_callback_t callback) { motion_state_callback_ = std::move(callback); }

  // Fired every `publish_interval` packets once isReady() is true.
  void onMotionUpdate(motion_update_callback_t callback) { motion_update_callback_ = std::move(callback); }

  // ---- Diagnostics --------------------------------------------------------

  const char* getDetectorName() const { return detector_ != nullptr ? detector_->get_name() : "none"; }
  const CSIManager& getCsiManager() const { return csi_manager_; }
  const uint8_t* getSelectedSubcarriers() const { return selected_subcarriers_; }

 private:
  void startCalibration_();
  void onWifiConnected_();
  void onWifiDisconnected_();

  EspectreConfig config_;

  BaseDetector* detector_{nullptr};
  MVSDetector mvs_detector_;
  MLDetector ml_detector_;

  CSIManager csi_manager_;
  WiFiLifecycleManager wifi_lifecycle_;
  NBVICalibrator nbvi_calibrator_;
  TrafficGeneratorManager traffic_generator_;
  UDPListener udp_listener_;

  uint8_t selected_subcarriers_[12]{};
  bool fixed_subcarriers_{false};

  float threshold_{1.0f};
  float best_pxx_{0.0f};  // last calibration baseline (Pxx x factor)

  MotionState motion_state_{MotionState::IDLE};
  bool ready_{false};

  motion_state_callback_t motion_state_callback_;
  motion_update_callback_t motion_update_callback_;
};

}  // namespace espectre
