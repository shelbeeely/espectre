/*
 * ESPectre Core + FreeInk SDK example
 *
 * Shows Wi-Fi CSI motion detection on a FreeInk e-paper device: a full-screen
 * status page that flips between "No motion" and "MOTION DETECTED" as
 * EspectreCore's detector fires, plus a one-line status while the sensor is
 * still gain-locking/calibrating after boot.
 *
 * Built against the Xteink X4 target (see platformio.ini in this directory);
 * BoardConfig::ACTIVE makes the same code portable to any other FreeInk
 * device by changing the -DFREEINK_DEVICE_* build flag.
 *
 * espectre-core owns Wi-Fi's CSI-specific configuration (protocol,
 * bandwidth, promiscuous mode) itself via EspectreCore::begin() - this
 * sketch only needs to associate to an AP with WiFi.begin().
 */

#include <Arduino.h>
#include <WiFi.h>

#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <FreeInkUIDisplayTarget.h>

#include <espectre_core.h>

// ---- Fill in your network -------------------------------------------------
static const char *WIFI_SSID = "your-ssid";
static const char *WIFI_PASSWORD = "your-password";
// ----------------------------------------------------------------------------

EInkDisplay display(BoardConfig::ACTIVE.display.sclk, BoardConfig::ACTIVE.display.mosi,
                     BoardConfig::ACTIVE.display.cs, BoardConfig::ACTIVE.display.dc,
                     BoardConfig::ACTIVE.display.rst, BoardConfig::ACTIVE.display.busy);

espectre::EspectreCore motion;

static freeink::ui::DisplayTarget *drawTarget = nullptr;
static int16_t screenWidth = 0;
static int16_t screenHeight = 0;

// Set from EspectreCore's callback (WiFi/CSI event context); only read/cleared
// from loop(), so a plain bool is enough here - no ISR involved.
static volatile bool needsRedraw = true;

void onMotionStateChanged(espectre::MotionState) {
  needsRedraw = true;
}

void renderStatus() {
  using namespace freeink::ui;

  const bool ready = motion.isReady();
  const bool active = motion.isMotionDetected();

  const Color bg = active ? Color::Black : Color::White;
  const Color fg = active ? Color::White : Color::Black;

  drawTarget->fill(Rect{0, 0, screenWidth, screenHeight}, Paint::solid(bg));

  const char *headline;
  if (!ready) {
    headline = motion.isCalibrating() ? "Calibrating..." : "Starting...";
  } else {
    headline = active ? "MOTION DETECTED" : "No motion";
  }

  TextStyle headlineStyle;
  headlineStyle.align = TextAlign::Center;
  headlineStyle.color = fg;
  headlineStyle.bold = true;
  drawTarget->text(Rect{0, static_cast<int16_t>(screenHeight / 2 - 24), screenWidth, 48}, headline, headlineStyle);

  char detail[64];
  if (ready) {
    snprintf(detail, sizeof(detail), "%s detector - level %.2f / %.2f", motion.getDetectorName(),
             motion.getMovementLevel(), motion.getThreshold());
  } else {
    snprintf(detail, sizeof(detail), "gain %s", motion.isGainLocked() ? "locked" : "locking...");
  }
  TextStyle detailStyle;
  detailStyle.align = TextAlign::Center;
  detailStyle.color = fg;
  drawTarget->text(Rect{0, static_cast<int16_t>(screenHeight / 2 + 30), screenWidth, 24}, detail, detailStyle);

  // Full refresh on a black<->white flip (avoids ghosting), fast refresh for
  // in-place metric updates while the state stays the same.
  static bool wasActive = false;
  const bool flipped = (wasActive != active);
  wasActive = active;
  display.displayBuffer(flipped ? EInkDisplay::FULL_REFRESH : EInkDisplay::FAST_REFRESH);
}

void setup() {
  Serial.begin(115200);

  display.begin();
  static freeink::ui::DisplayTarget targetInstance(display.getFrameBuffer(), display.getDisplayWidth(),
                                                     display.getDisplayHeight(), display.getDisplayWidthBytes());
  drawTarget = &targetInstance;
  screenWidth = targetInstance.deviceContext().width;
  screenHeight = targetInstance.deviceContext().height;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  espectre::EspectreConfig config;  // defaults: MVS detector, AUTO threshold, NBVI auto-calibration
  // Only redraw on IDLE<->MOTION transitions: e-paper refreshes are slow and
  // visibly flash, so this deliberately does not also redraw on every
  // onMotionUpdate() tick (which fires every `publish_interval` packets,
  // ~1/s) even though that callback carries the live movement level too.
  motion.onMotionStateChanged(onMotionStateChanged);
  if (motion.begin(config) != ESP_OK) {
    Serial.println("EspectreCore::begin() failed - check Serial log above for the ESP-IDF error");
  }

  renderStatus();
}

void loop() {
  motion.loop();

  if (needsRedraw) {
    needsRedraw = false;
    renderStatus();
  }

  delay(50);
}
