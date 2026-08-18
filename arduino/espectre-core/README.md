# espectre-core

A standalone, Arduino-native port of [ESPectre](https://github.com/francescopace/espectre)'s
Wi-Fi CSI motion-detection engine. ESPectre itself ships as an
[ESPHome](https://esphome.io/) external component, tightly coupled to
ESPHome's `Component`/`Sensor`/`Switch`/`Number` framework and YAML codegen so
it can publish directly to Home Assistant. `espectre-core` is the same
detection pipeline with that layer stripped out, so it builds under the plain
Arduino-ESP32 framework and drops into any PlatformIO project - including
firmware built on other Arduino-ESP32 SDKs such as
[FreeInk](https://github.com/Free-Ink/freeink-sdk) (see
`examples/freeink_motion_display`).

## What's ported, and what isn't

Ported (this directory):

- `csi_manager` - ESP32 WiFi CSI hardware configuration and packet pipeline
- `mvs_detector` / `ml_detector` - the two motion-detection algorithms
  (Moving Variance Segmentation, and a small on-device MLP), plus their
  shared `base_detector` buffer/filter management
- `nbvi_calibrator` - automatic subcarrier band selection + adaptive
  threshold calibration
- `gain_controller` - AGC/FFT gain locking for stable CSI amplitudes
- `wifi_lifecycle` - the CSI-specific WiFi protocol/bandwidth/promiscuous-mode
  setup and connect/disconnect event handlers
- `traffic_generator_manager` / `udp_listener` - synthetic ping/DNS traffic
  (or an external UDP trigger) to keep CSI packets flowing at a steady rate;
  without one of these, CSI packets only arrive as a side effect of whatever
  WiFi traffic already exists, which is normally far too sparse for reliable
  detection
- `espectre_core.{h,cpp}` - new: a minimal orchestrator (`EspectreCore`)
  that wires all of the above together behind a `begin()`/`loop()` API,
  replacing the ESPHome component's `ESpectreComponent::setup()`/`loop()`

Not ported (still ESPHome/Home-Assistant-only, live in `components/espectre/`
in the main repo):

- `espectre.{h,cpp}` (`ESpectreComponent`) - the ESPHome `Component` itself,
  YAML config setters, and BLE telemetry channel
- `sensor_publisher`, `threshold_number`, `calibrate_switch` - HA
  sensor/number/switch publishing

If you want Home Assistant integration, use the ESPHome component directly;
this library is for firmware that has no ESPHome/HA in the picture at all.

## Usage

```cpp
#include <WiFi.h>
#include <espectre_core.h>

espectre::EspectreCore motion;

void setup() {
  WiFi.mode(WIFI_STA);
  WiFi.begin("ssid", "password");

  // Configures CSI-critical WiFi params (protocol/bandwidth/promiscuous
  // mode) and registers connect/disconnect handlers. Detection starts once
  // WiFi associates - begin() itself does not block on a connection.
  motion.begin();
}

void loop() {
  motion.loop();  // drains the UDP listener socket in external-traffic mode

  if (motion.isReady() && motion.isMotionDetected()) {
    // ...
  }
}
```

See `espectre_core.h` for the full `EspectreConfig` options (detector choice,
window size, filters, gain-lock mode, fixed vs. auto-calibrated subcarriers,
traffic generator rate/mode) and `EspectreCore`'s accessors
(`getMovementLevel()`, `isCalibrating()`, `isGainLocked()`,
`onMotionStateChanged()`, `onMotionUpdate()`, `setThreshold()`,
`triggerCalibration()`).

## Requirements

- ESP32 with CSI support (S3/C6 recommended; C3, C5, and original ESP32 also
  work - see the main repo's `SETUP.md` for the platform comparison), built
  with the **Arduino framework** (tested against `pioarduino`'s
  `platform-espressif32`, Arduino core 3.3.x / ESP-IDF 5.5.x, matching what
  FreeInk itself builds against).
- A partition scheme with a **SPIFFS partition** - the NBVI calibrator
  buffers packets to `/spiffs/nbvi_buffer.bin` during band selection.
  PlatformIO's `default` or `min_spiffs` partition schemes both work; you do
  not need `components/espectre/partitions.csv` (that table also carves out
  OTA partitions ESPHome uses, which this library does not need).
- `CONFIG_IDF_TARGET_ESP32<S3|C3|C5|C6>` gain-lock support requires linking
  against Espressif's PHY blob, which the standard Arduino-ESP32 core already
  does - no extra configuration needed.

## Adding to a PlatformIO project

```ini
lib_deps =
  espectre-core=symlink:///path/to/espectre/arduino/espectre-core
```

(or vendor/submodule this directory and point `lib_deps` at it, or publish it
to the PlatformIO registry under your own name).

## Example: FreeInk e-paper motion display

`examples/freeink_motion_display` shows `EspectreCore` driving a full-screen
status page on a FreeInk e-paper device (Xteink X4 by default; swap the
`-DFREEINK_DEVICE_*` build flag for another supported board) via FreeInk's
`EInkDisplay` + `FreeInkUI` `DisplayTarget`. The [FreeInk SDK](https://github.com/Free-Ink/freeink-sdk)
is vendored as a git submodule at `arduino/freeink-sdk`; fetch it with:

```sh
git submodule update --init arduino/freeink-sdk
```

(or clone this repo with `git clone --recursive` in the first place).

## Caveats from the port

- This has been reviewed for correctness against the ESPHome component's
  logic but **not build- or hardware-tested** in this environment (no ESP32
  Arduino toolchain was available here). Please build and flash it before
  relying on it, and open an issue against the main repo with anything that
  doesn't compile or behave as documented above.
- The ML detector's weights (`ml_weights.h`) are copied unchanged from the
  ESPHome component; they were trained against the same CSI processing
  pipeline ported here, so behavior should be identical.
- `esp_log.h`'s `ESP_LOGx` macros are used directly in place of ESPHome's
  logging wrapper (`esphome/core/log.h`) - which itself just forwards to
  `esp_log.h` under the hood - so log output/format is unchanged, but log
  level filtering follows Arduino-ESP32's own `CORE_DEBUG_LEVEL` /
  `esp_log_level_set()` instead of ESPHome's `logger:` YAML config.
