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

## 2021-03-25

- `fw/8bit-quad-fc` rebranded/reorganized: renamed namespaces
  (`pid::inner/outer` -> `ctlr::att::rate/angle`,
  `config::regulation` -> `config::ctlr::att::rate/angle`). The rename itself
  is dated: `config.hpp` was last written 2021-03-24, the day before the
  snapshot this revision is taken from.
- I2C clock settled on a single 800 kHz call. The 2018 revision set 400 kHz
  early in `setup()` and 800 kHz again later, so the effective rate was
  already 800 kHz but stated twice.
- Retuned: `yawI` 0.03 -> 0.05, `battery::alpha` 0.98 -> 0.7, and a new
  `yaw_updateRate` setting for the yaw rate PID.
- Pressure sampling gets a 3-sample median filter (`middle_of_3`) before its
  existing low-pass filter, and boot-time pressure calibration is tightened
  (faster warm-up, averaged samples) - an accuracy improvement to the read
  path itself, independent of what's done with the resulting altitude value.
- Removed the unused/never-engaged `MPU6050` driver (`src/imu/MPU6050.{hpp,
  cpp}`) - `MPU9255` is the only IMU from this point on.
- Removed the empty, never-included `cfg.hpp` scaffolding stub, the
  `DEBUG_ACCELEROMETER_ANGLE` switch, whose only use site went with the
  complementary-filter code, and large blocks of superseded commented-out code (an old EEPROM climb/fall-speed
  black-box logger, stale debug prints, a pasted raw calibration data log)
  that had accumulated in `main.ino`/`MPU9255.cpp`.
- Altitude-hold is not carried into this repository. The source snapshot has
  it wired up (a vertical-speed PID engaged via `Command.altitudeHold`), but
  it was never a working feature, and the author himself commented it out
  nine days later, so it is left out from here rather than carried and then
  removed. `Command.altitudeHold` and the `SettingId::regAlt_p`/`regAlt_d`
  enum values stay in `comm_packets.hpp` for wire-protocol numbering
  compatibility with `fw/8bit-quad-rc`, but nothing on the `fc` side reads or
  handles them (a setting request for either falls through to
  `handleSetting()`'s `default` case). Basic barometer altitude computation
  (`relativeAltitude`, reported in `Telemetry`) is kept, since it existed
  independent of the hold feature.
- `fw/8bit-quad-rc` is unchanged at this point in history.
- `fw/tool`: `calib-imu` updated to this revision's own copy of the
  offset-finder, which adds sampling delays and narrows its search
  dynamically; the previous commit carries a 2018 copy of the same sketch.
  Added `trim-imu`, a bench sketch for dumping raw IMU readings (MPU6050 or
  MPU9255, selectable) to help manually trim offsets.
- Not carried over from the source snapshot, alongside altitude-hold:
  - The complementary-filter attitude estimator, an alternative to Mahony
    behind `#if FUSION_COMPLEMENTARY_FILTER`. It could not be selected as
    found: `config::imu::complementary::alpha`, `oneMinusAlpha` and
    `config::imu::epsilon` are all commented out in the source's
    `config.hpp`, so enabling it would not compile. `MAHONY_FUSION` is left
    as the sole, always-true branch. The 2018 revision kept the same code
    commented out in the sketch; the source restructured it into a live
    `#if` here without restoring what it needs.
  - The `ANGULAR_RATE_DIRECT_FROM_GYROSCOPE` /
    `ANGULAR_RATE_DIFFERENTIATE_FROM_MAHONY` switch. The selected branch
    (differentiate) is kept as plain code; the unselected one-line gyro read
    and the switch itself are dropped.
  - Six debug blocks that were live but switched off: `DEBUG_EEPROM`,
    `BLACK_BOX_HOVER_VOLTAGE`, `PRINT_BATTERY_VOLTAGE`, `PRINT_THROTTLE`,
    `DEBUG_ANGLE_CTRL_PERIOD`, `DEBUG_RATE_CTRL_PERIOD`.
  - `DEBUG_PIN` and the four `DBG_PIN_*` defines, which the source defines
    but never references anywhere in the sketch.
- The yaw rate's `dt` is `uint32_t` here, matching the source. At a 3100 us
  cycle it truncates to zero and `avYaw` divides by zero. Kept rather than
  quietly fixed: it is a regression against the 2018 revision, which used a
  float, and the next revision is where it goes away.
