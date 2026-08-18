/*
 * ESPectre Core - Arduino-native orchestrator implementation
 *
 * Adapted from the ESPHome external_component's espectre.cpp
 * (ESpectreComponent::setup/on_wifi_connected_/start_calibration_), with the
 * Home Assistant sensor/switch/number and BLE telemetry glue removed.
 *
 * Author: Francesco Pace <francesco.pace@gmail.com>
 * License: GPLv3
 */

#include "espectre_core.h"

#include <cstring>

#include "esp_log.h"

namespace espectre {

static const char *TAG = "EspectreCore";

esp_err_t EspectreCore::begin(const EspectreConfig &config) {
  config_ = config;

  ESP_LOGI(TAG, "Initializing ESPectre Core...");

  // 0. Initialize WiFi for optimal CSI capture (protocol/bandwidth/promiscuous mode).
  esp_err_t wifi_init_err = wifi_lifecycle_.init();
  if (wifi_init_err != ESP_OK) {
    ESP_LOGE(TAG, "WiFi lifecycle init failed: %s", esp_err_to_name(wifi_init_err));
    return wifi_init_err;
  }

  // 1. Configure subcarriers: fixed (user-supplied) or default until NBVI calibration runs.
  if (config_.fixed_subcarriers != nullptr) {
    memcpy(selected_subcarriers_, config_.fixed_subcarriers, 12);
    fixed_subcarriers_ = true;
  } else {
    memcpy(selected_subcarriers_, DEFAULT_SUBCARRIERS, 12);
    fixed_subcarriers_ = false;
  }

  // 2. Configure the motion detector based on algorithm selection.
  if (config_.algorithm == DetectionAlgorithm::ML) {
    float ml_threshold =
        (config_.threshold_mode == ThresholdMode::MANUAL) ? config_.manual_threshold : ML_DEFAULT_THRESHOLD;
    threshold_ = ml_threshold;
    ml_detector_ = MLDetector(config_.window_size, ml_threshold);
    ml_detector_.configure_lowpass(config_.lowpass_enabled, config_.lowpass_cutoff_hz);
    ml_detector_.configure_hampel(config_.hampel_enabled, config_.hampel_window, config_.hampel_threshold);
    detector_ = &ml_detector_;
    ESP_LOGI(TAG, "Using ML detector (window=%d, threshold=%.2f)", config_.window_size, ml_threshold);
  } else {
    threshold_ = config_.manual_threshold;
    mvs_detector_ = MVSDetector(config_.window_size, threshold_);
    mvs_detector_.configure_lowpass(config_.lowpass_enabled, config_.lowpass_cutoff_hz);
    mvs_detector_.configure_hampel(config_.hampel_enabled, config_.hampel_window, config_.hampel_threshold);
    detector_ = &mvs_detector_;
    ESP_LOGI(TAG, "Using MVS detector (window=%d, threshold=%.2f)", config_.window_size, threshold_);
  }

  // 3. Initialize NBVI calibrator, traffic generator, and UDP listener (external traffic mode).
  nbvi_calibrator_.init(&csi_manager_);
  nbvi_calibrator_.set_mvs_window_size(config_.window_size);
  nbvi_calibrator_.configure_lowpass(config_.lowpass_enabled, config_.lowpass_cutoff_hz);
  nbvi_calibrator_.configure_hampel(config_.hampel_enabled, config_.hampel_window, config_.hampel_threshold);
  nbvi_calibrator_.set_buffer_size(config_.window_size * CALIBRATION_NUM_WINDOWS);

  traffic_generator_.init(config_.traffic_generator_rate_pps, config_.traffic_generator_mode);
  udp_listener_.init(config_.udp_listen_port);

  // 4. Initialize CSI manager with the selected detector.
  csi_manager_.init(detector_, selected_subcarriers_, config_.publish_interval, config_.gain_lock_mode);
  csi_manager_.set_evaluation_interval(config_.evaluation_interval);
  csi_manager_.set_motion_on_hits(config_.motion_on_hits);
  csi_manager_.set_motion_off_hits(config_.motion_off_hits);

  // 5. Register WiFi lifecycle handlers (drives gain lock -> calibration -> ready on connect).
  esp_err_t handlers_err = wifi_lifecycle_.register_handlers([this]() { this->onWifiConnected_(); },
                                                              [this]() { this->onWifiDisconnected_(); });
  if (handlers_err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register WiFi handlers: %s", esp_err_to_name(handlers_err));
    return handlers_err;
  }

  ESP_LOGI(TAG, "ESPectre Core initialized successfully");
  return ESP_OK;
}

void EspectreCore::onWifiConnected_() {
  motion_state_ = MotionState::IDLE;

  csi_manager_.set_motion_state_callback([this](MotionState state) {
    motion_state_ = state;
    if (motion_state_callback_) {
      motion_state_callback_(state);
    }
  });

  if (!csi_manager_.is_enabled()) {
    ESP_ERROR_CHECK(csi_manager_.enable([this](MotionState state, uint32_t packets_received) {
      motion_state_ = state;
      if (!ready_) {
        return;
      }
      if (motion_update_callback_) {
        motion_update_callback_(state, getMovementLevel(), packets_received);
      }
    }));
  }

  // Start synthetic traffic, or the UDP listener for externally-driven traffic.
  if (config_.traffic_generator_rate_pps > 0) {
    if (!traffic_generator_.is_running()) {
      if (!traffic_generator_.start()) {
        ESP_LOGW(TAG, "Failed to start traffic generator");
      }
    }
  } else {
    if (!udp_listener_.is_running()) {
      if (!udp_listener_.start()) {
        ESP_LOGW(TAG, "Failed to start UDP listener");
      }
    }
  }

  // Two-phase calibration once WiFi is up: gain lock, then NBVI/baseline calibration.
  csi_manager_.set_gain_lock_callback([this]() {
    auto &gc = csi_manager_.get_gain_controller();
    bool need_cv = gc.needs_cv_normalization();
    detector_->set_cv_normalization(need_cv);
    nbvi_calibrator_.set_cv_normalization(need_cv);
    startCalibration_();
  });

  ready_ = true;
}

void EspectreCore::onWifiDisconnected_() {
  csi_manager_.disable();
  motion_state_ = MotionState::IDLE;

  if (traffic_generator_.is_running()) {
    traffic_generator_.stop();
  }
  if (udp_listener_.is_running()) {
    udp_listener_.stop();
  }

  ready_ = false;
}

void EspectreCore::loop() {
  if (udp_listener_.is_running()) {
    udp_listener_.loop();
  }
}

void EspectreCore::setThreshold(float threshold) {
  threshold_ = threshold;
  csi_manager_.set_threshold(threshold);
  ESP_LOGD(TAG, "Threshold updated to %.2f (session-only)", threshold);
}

void EspectreCore::startCalibration_() {
  // ML detector uses fixed subcarriers from training - no band calibration needed.
  if (config_.algorithm == DetectionAlgorithm::ML) {
    memcpy(selected_subcarriers_, DEFAULT_SUBCARRIERS, 12);
    csi_manager_.update_subcarrier_selection(DEFAULT_SUBCARRIERS);
    ESP_LOGI(TAG, "ML detector - using default subcarriers, skipping calibration");
    return;
  }

  auto calibration_callback = [this](const uint8_t *band, uint8_t size, const std::vector<float> &cal_values,
                                      bool success) {
    if (success && !fixed_subcarriers_) {
      memcpy(selected_subcarriers_, band, size);
      csi_manager_.update_subcarrier_selection(band);
    }

    if (band != nullptr && !cal_values.empty()) {
      float adaptive_threshold;
      uint8_t percentile;
      calculate_adaptive_threshold(cal_values, config_.threshold_mode, adaptive_threshold, percentile);
      best_pxx_ = adaptive_threshold;

      if (config_.threshold_mode != ThresholdMode::MANUAL) {
        setThreshold(adaptive_threshold);
        ESP_LOGD(TAG, "Adaptive threshold: %.4f (P%d)", adaptive_threshold, percentile);
      }

      csi_manager_.clear_detector_buffer();
    }

    traffic_generator_.resume();
    ESP_LOGD(TAG, "Calibration %s", success ? "completed successfully" : "failed");
  };

  nbvi_calibrator_.set_collection_complete_callback([this]() { traffic_generator_.pause(); });

  esp_err_t err = nbvi_calibrator_.start_calibration(selected_subcarriers_, 12, calibration_callback);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start calibration: %s", esp_err_to_name(err));
  }
}

void EspectreCore::triggerCalibration() {
  if (nbvi_calibrator_.is_calibrating()) {
    ESP_LOGW(TAG, "Calibration already in progress");
    return;
  }
  if (!csi_manager_.is_gain_locked()) {
    ESP_LOGW(TAG, "Cannot recalibrate: gain not yet locked");
    return;
  }
  ESP_LOGI(TAG, "Manual recalibration triggered");
  startCalibration_();
}

}  // namespace espectre
