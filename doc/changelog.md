# Changelog

Code/architecture history only. Physical-build flight-tuning notes (gains,
ESC settings, hardware fixes for one specific airframe) are intentionally
not tracked here - see `doc/history.md` for why. Superseded IMU calibration
sets, which the source carried as commented-out code, are omitted from the
firmware for the same reason.

## 2018-05-04

- Added the custom "8bit-quad" Arduino core (`hardware/`): swaps Timer0/
  Timer2 so 4 PWM outputs are available at a consistent frequency for ESCs
  (moves the `millis()`/`micros()` time base off Timer0 and onto Timer2,
  freeing Timer0 to be reconfigured for PWM instead of being locked to its
  stock timekeeping-driven mode), defines `8bit-quad_fc`/`8bit-quad_rc`
  board targets for the atmega328p. `package_8bit-quad_index.json` adds a
  Boards Manager install path alongside the manual sketchbook-copy one; it
  points at the published `hardware-v1.0.0` archive.
- Added `fw/8bit-quad-fc`: earliest working flight-controller firmware.
  nRF24L01+ link, MPU9255 IMU, Mahony sensor fusion, cascaded angle (outer)
  + rate (inner) PID, EEPROM-backed tunable settings, EEPROM black-box
  logging of battery/current, altitude-hold code present but never engaged.
  `fw/tool/calib-imu`, `fw/tool/calib-escs-range` and
  `fw/tool/test-radio-link` are independent bench sketches, not part of this
  build. `test-radio-link` is the oldest surviving code in the project
  (2018-02-05): one sketch flashed to both ends with `bool drone` deciding
  which pipe is which, sending `Command` and `Setting` packets on timers and
  printing whatever comes back. It is how the wire protocol was brought up.
- Added `fw/8bit-quad-rc`: the matching remote-transmitter firmware. Its only
  surviving copy sits in a later snapshot, but it is period-accurate rather
  than backfilled: its files were last written between 2018-01-23 and
  2018-04-26, it was never modified afterwards, and its wire protocol header
  differs from `fc`'s by a trailing newline only.
- Communication packet structs (`Command`/`Setting`/`Telemetry*`) and IMU
  value types (`AngularVelocity`/`Acceleration`/`Attitude`), used
  identically by both `fc` and `rc`, are kept as a duplicated `common/`
  folder in each project (`fw/8bit-quad-fc/src/common/`,
  `fw/8bit-quad-rc/src/common/`) rather than a single shared file: Arduino
  sketches can't include files from outside their own folder tree via
  `../`, and a proper shared library was more machinery than this needed.
- Every `.cpp`/`.hpp` module in both `fc` and `rc` lives under each
  project's `src/` folder. Arduino only auto-compiles files that are either
  directly in the sketch root or recursively under a `src/` subfolder of
  it; a plain top-level subfolder (what this started as) is silently
  ignored.
- This revision depends on external libraries that are no longer
  distributed through Arduino's Library Manager (`UART_atmega328`, a
  class-based `MahonyAHRS`, `MPU925x_I2C`) alongside ones that still are
  (`MPU6050`, `RF24`, the BMP280 driver `I2C-Sensor-Lib_iLib`). All six are
  vendored inside the core, under `hardware/8bit-quad/avr/libraries/`, so
  the build doesn't depend on what's currently installed in anyone's
  Arduino Sketchbook:
  - `MPU925x_I2C` frozen at its own commit `15e9013`, dated
    2018-05-04 (matches this commit's date).
  - `MahonyAHRS` frozen at its own commit `52fc1f5`, dated
    2018-12-21 (about 7 months after this commit's date; it's the only copy
    available, not a perfect period-accurate match). Its class API
    (`Mahony(samplePeriod)`, `update(yaw, pitch, roll, ax,ay,az, gx,gy,gz)`
    with gyro in rad/s and outputs in radians) differs from what
    `8bit-quad-fc.ino` originally assumed (a no-arg constructor, a
    `begin(sampleRate)` call, an `updateIMU(gx,gy,gz,ax,ay,az)` taking
    degrees/s, and `getPitch()`/`getRoll()`/`getYaw()` getters) - adapted
    the call site to this library's real interface (unit conversions via
    `DEG_TO_RAD`/`RAD_TO_DEG`) rather than changing the vendored library.
  - The other four don't have their own git history available to pin to a
    commit; vendored as installed.
  - `fw/8bit-quad-fc/src/imu/MPU6050.{hpp,cpp}` is vendored but unused in
    this revision (only `MPU9255` is wired up) - it depends on the vendored
    `MPU6050` library's own `I2Cdev.h`/`.cpp` rather than a second, sketch-
    local copy, to avoid a duplicate-symbol link error.
  - `fw/8bit-quad-fc/src/control/Altitude.hpp` and
    `fw/8bit-quad-rc/src/config/EepromSetting.hpp` are unused stub/dead
    files carried over as-is.
