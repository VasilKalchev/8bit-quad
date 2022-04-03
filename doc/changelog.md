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

## 2021-04-03

- `MPU9255::getMotion()` now returns a status code (0 ok, 1 zero-read, 2 I2C
  failure) instead of `void`; `loop()` counts zero-reads and I2C failures
  instead of silently ignoring them.
- I2C clock forced back down to 400 kHz with a direct `TWBR = 12` after the
  `Wire.setClock(800000)` call, which overrides it. The 800 kHz attempt did
  not stick; the 2022 revision raises it again.
- `MPU925x_I2C`: the read timeout is now measured with `micros()`
  and set to 1000 us, instead of `millis()` and 11 ms. At a 3100 us cycle the
  old value could stall the loop for three cycles on one bad read, which is
  the failure this revision's read-status reporting exists to catch. This is
  a local edit to the vendored library, found in the snapshot and never
  committed upstream, so it no longer matches the pinned `15e9013`.
- Recalibrated the accelerometer and gyro offsets.
- `yawI` 0.05 -> 0.04.
- The rest of this window's changes in the source are altitude-hold being
  commented out and a compile-time IMU selection macro being added, neither of
  which is carried into this repository.
- Yaw rate is now read straight from the gyro (`avYaw = angularVelocity.z`)
  instead of being differentiated from the fused yaw angle, which also
  removes the previous revision's `uint32_t` dt and its division by zero.
  The source kept differentiating until the 2022 revision, where it made the
  same change; brought forward here rather than carrying a broken yaw rate
  through another revision.

## 2022-04-03

The last revision that flew. Written between 2021-12-20 and 2022-04-03,
after eight months of nothing.

- The nRF24L01+ link is replaced by a standard PPM receiver on `D2`. New
  `src/rx/`: `ReadRx()` pulls eight channels, `SplitChannel()` decodes two
  logical switches from one channel by value bracket, `CombineChannels()`
  packs four 3-position switches into a 12-bit mask. `PPMReader` (GPL-3.0,
  Aapo Nikkila / Dmitry Grigoryev) is vendored in the sketch rather than
  with the core's libraries, which keeps it out of the distributed core
  archive.
- `fw/8bit-quad-rc` is removed. The wire protocol it exists for is gone, so
  the transmitter has nothing to talk to and no shared code left. It is still
  in the previous commits, unchanged since 2018.
- Removed with it: `src/radio/`, `src/common/comm_packets.hpp` and the whole
  `src/common/` folder, which only existed to be duplicated into `rc`.
  `imu_types.hpp` moves to `src/imu/`.
- Vendored libraries: dropped `RF24` (only `rc` and
  `fw/tool/test-radio-link` used it), `MahonyAHRS` (the sketch now carries its own) and
  `I2C-Sensor-Lib_iLib` (no barometer here). What is left is what the tree
  builds against. All three are still in the earlier commits, and
  `hardware-v1.0.0` is tagged there.
- `fw/tool/test-radio-link` goes with the radio, for the same reason.
- A third `direct` flight mode is in the code alongside `angle` and `acro`,
  driving the motor mix outputs from the sticks with every PID in `MANUAL`.
  It was a one-off test and was never flown.
- Configuration moves onto the RC switches. Four 3-position switches encoded
  across two PPM channels select flight mode, lights, accelerometer and gyro
  calibration and eight tuning slots; two potentiometers supply the value.
  The map is `cfg::rx::switch_state::*` in `src/config/config_rx.hpp`. Only
  the two calibrations and the mode select are implemented; the tuning slots
  print their name and do nothing.
- In-flight offset trimming: with the accel-calibrate combo held, pitch/roll
  stick nudges `setXAccelOffset`/`setYAccelOffset`; with the gyro combo held
  and the throttle down, three-sample averages walk the gyro offsets toward
  zero one count at a time.
- Rate loop is full PID for pitch and roll, D at 0.07. It was P-only from
  2018 until here. Angle loop stays PI.
- Retimed: sub-cycle 2600 us so the rate loop runs at ~385 Hz (was 3100 us,
  ~322 Hz), full cycle 10400 us, angle loop 10560 us (was 12406). The
  sub-cycle map is now rc / battery + indication / quaternion to Euler /
  angle controller.
- I2C back up to 800 kHz, this time by `Wire.setClock()` alone with the
  `TWBR` override removed, so it sticks.
- The MPU6050 is the live IMU again, selected by `#define IMU_MPU6050`;
  `MPU9255` is kept and buildable via `IMU_MPU925x`. The MPU6050 driver
  returns to the tree, having been dropped in 2021-03-25.
- Namespace `config::` renamed to `cfg::` throughout. File names are
  unchanged, so `src/config/config.hpp` now opens `namespace cfg`.
- The `EepromSetting<T>` framework is gone. Six PID gains are written and
  read as raw floats at fixed addresses through `avr/eeprom.h`, each
  validated against a hardcoded plausible range on read. `config.cpp` keeps
  the full EEPROM map as a comment, including regions nothing uses yet.
  `RESET_EEPROM_TO_DEFAULT` is `true` as found, which overwrites the stored
  gains with the compiled-in ones at every boot.
- Fusion: the sketch now carries its own copy of the class-based
  `MahonyAHRS` (`src/fusion/MahonyAHRS.{cpp,h}`), which is what the 2018 code
  had always assumed. It replaces both the unused C globals version that sat
  there and the vendored library the call site had been adapted to. Sample
  frequency 384 Hz, `twoKp` 0.65, `twoKi` 0.01.
- The complementary filter is back as a live `#if` alternative to Mahony
  (`ATTITUDE_FUSION_METHOD_COMPLEMENTARY`), and this time the config it needs
  exists, so it compiles. Mahony is the selected one.
- Altitude and the barometer are gone entirely: no BMP280 read, no relative
  altitude, no telemetry to report it on.
- Debug output moved from `uart::` to Arduino `Serial` at 2 Mbaud, behind the
  same `PRINT()` macro. `src/util/debug.hpp` grows a per-topic switch list
  and a `TIME_START`/`TIME_STOP` pair that buffers timings and dumps them in
  batches.
- `utils` moves into `namespace util`; `middle_of_3` becomes overloaded
  `util::Median` for `int16_t` and `float`.
- Local edits to the vendored libraries, found in the snapshot and never
  committed upstream:
  - `MPU6050`: `initialize()`, `getMotion6()` and the six offset setters
    return `bool` instead of `void`, so I2C failures reach the caller. This
    is what the sketch's new zero-read and read-failure counters need, and
    it mirrors what `MPU9255::getMotion()` got in the previous commit.
  - `MPU6050`: `I2Cdev` switched from `I2CDEV_BUILTIN_FASTWIRE` to
    `I2CDEV_ARDUINO_WIRE`, which is why `setup()` now calls `Wire.begin()`.
  - `UART_atmega328`: gains `available()`, and `read()` no longer blocks
    waiting for a byte.
- `fw/tool`: `calib-imu` and `trim-imu` updated to this revision's copies.
  Added `test-motors` (ramps the four motors), `bench-motor` (an off-board
  single-motor rig with an LCD, a pot, a kill switch and ESC voltage
  readout, needs `LiquidCrystal I2C` from Library Manager) and
  `calib-balance` (RC-driven IMU calibration, ESC range and per-motor RMS
  acceleration for balancing props).
- Fixed one build error in the snapshot: `readBatteryVoltage()` cast to
  `float32_t`, which nothing defines. The cast was `float` until 2022-02-25
  and the file was not compiled again after that edit.
- Not carried: `src/obsolete.cpp`, 412 lines of commented-out fragments that
  would not compile as a translation unit; `cfg/cfg_ctrlr.hpp` and
  `cfg/cfg_imu.hpp`, declaration-only stubs of a config split that was never
  finished and never included; a stale duplicate `src/board/` from an earlier
  move; and `IMU_zero (dep)`, the author's own name for the bench sketch that
  `calib-imu` had already replaced.
- IMU calibration sweeps pasted into `MPU6050.cpp`, `MPU9255.cpp`,
  `config.cpp` and `trim-imu` are omitted, as in the earlier revisions. The
  datasheet tables in the same comment blocks are kept.
- Two wish lists that sat at the top of `8bit-quad-fc.ino` and
  `fw/tool/bench-motor` are not carried. They were the first thing either file
  showed and none of it was started; the notable entry is that there is still
  no arm/disarm.
- `gallery/`: thirteen photos from this revision's window, the last of them
  taken 2022-12-10, eight months after the code stopped. The contact sheet is
  regenerated to include them.
