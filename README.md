# pypilot-sensors

C++ sensor acquisition and connector-adapter layer for modular pypilot.

This project converts raw/native sensor input and decoded connector data into `pypilot-data-model` updates. It is intentionally below the runtime daemon and above protocol/connector modules.

## Dependencies

`pypilot-sensors` depends on:

* `pypilot-data-model`
* `pypilot-algorithms`
* `pypilot-nmea0183-connector`
* `pypilot-signalk-connector`
* `pypilot-servo-protocol`

The dependency direction is:

```text
pypilot-nmea0183-connector ┐
pypilot-signalk-connector ─┼──> pypilot-sensors ───> pypilot-data-model
pypilot-servo-protocol ────┘
```

`pypilot-sensors` does **not** depend on `pypilot-pilots-logic` or `pypilot-servo-runtime`.

## Responsibilities

This module owns:

* timestamped sensor sample structs
* DataModel writing for IMU/GPS/APB/wind/water/rudder/servo telemetry
* NMEA 0183 connector adapter
* Signal K connector adapter
* servo/rudder telemetry adapter using `pypilot-servo-protocol`
* replay adapter for deterministic tests and fixtures

This module does **not** own:

* autopilot mode selection
* pilot command computation
* servo command output runtime
* persistent settings storage
* user interface/server API

## Build

```bash
cmake -S . -B build \
  -DPYPILOT_DATA_MODEL_DIR=$PWD/../pypilot-data-model/src \
  -DPYPILOT_ALGORITHMS_DIR=$PWD/../pypilot-algorithms/src \
  -DPYPILOT_NMEA0183_CONNECTOR_DIR=$PWD/../pypilot-nmea0183-connector/src \
  -DPYPILOT_SIGNALK_CONNECTOR_DIR=$PWD/../pypilot-signalk-connector/src \
  -DPYPILOT_SERVO_PROTOCOL_DIR=$PWD/../pypilot-servo-protocol/src
cmake --build build
ctest --test-dir build --output-on-failure
```

## Arduino

Use the Arduino Library Manager or local `--libraries` paths for all dependencies:

```bash
arduino-cli compile --fqbn arduino:avr:mega \
  --libraries pypilot-sensors \
  --libraries pypilot-data-model \
  --libraries pypilot-algorithms \
  --libraries pypilot-nmea0183-connector \
  --libraries pypilot-signalk-connector \
  --libraries pypilot-servo-protocol \
  pypilot-sensors/examples/arduino/SensorsExample
```

## Current scope

This is a scaffold for later GitHub push. It is intentionally focused on connector-to-DataModel wiring and deterministic tests. Native hardware drivers can be added later without changing the public sample/writer API.
