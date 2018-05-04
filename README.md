# 8bit-quad, abandoned attempt: MPU6050 DMP attitude

**This branch is a dead end, kept for the record.** It was never flown, and
the flight path in it is a generation behind `main`. `main` is the working
firmware.

Forked from `nrf24-baseline`, where this sketch sat parked beside the working
one under the name `FC2.inoDMP`. The non-standard extension is what kept the
IDE from building it: two sketches cannot live in one Arduino folder, so
changing the extension was how the alternative was shelved without deleting
it.

## The idea

**Let the IMU do the fusion.** The MPU6050 has an on-chip Digital Motion
Processor that outputs a fused quaternion, so the ATmega328 would not have to
run a filter at all - it would read attitude over I2C the way it reads any
other register. On a chip with no FPU and a fixed-period loop to hold, moving
sensor fusion off the MCU is worth a look.

The DMP path is wired up in full: FIFO overflow handling, the data-ready
interrupt status, then `dmpGetQuaternion`, `dmpGetGravity` and
`dmpGetYawPitchRoll`.

## What it actually is

An evaluation harness, not a replacement. The DMP result never reaches the
controller. It is converted to degrees and printed next to the roll the
complementary filter produced, one CSV line per cycle:

```c
droll = ypr[1] * 180 / M_PI;
Serial.print(droll);
Serial.print(",");
Serial.println(attitude.roll);
```

So the sketch flies on the complementary filter while streaming both estimates
for comparison. `dpitch` is declared next to `droll` and never assigned, which
places this at the point where only one axis had been checked.

## Why it stopped

Mahony won. There is no note saying so, but the evidence is in the sketch:
this one has no Mahony include and estimates attitude with a complementary
filter, while the sketch that shipped alongside it at the same commit runs
Mahony behind `#define MAHONY_FUSION true`. This is the earlier fusion
generation, kept around long enough to be measured against its replacement and
then shelved.

The DMP also has a practical cost that shows in the code: attitude arrives
only when the FIFO says a packet is ready, and the read blocks on
`while (fifoCount < packetSize)`. That does not fit a loop holding a fixed
period as comfortably as filtering samples the loop already fetches.

## It does not compile

100 errors, all one root cause: the sketch is written against the generation
of `config::` that the baseline had already moved past. Every declaration it
needs is still in `src/config/config.hpp` at the fork point, commented out:

- `config::imu::complementary::{alpha,oneMinusAlpha}` and `config::imu::epsilon`,
  which the complementary filter needs and Mahony does not.
- The `config::imu::lowPassFilter` sub-namespaces `common`, `angularVelocity`
  and `acceleration`.
- `minimumRegulationThrottle` and `maximumBaseThrottle` as
  `EepromSetting<uint8_t>`. The baseline redeclared both as plain
  `const uint8_t`, so the sketch's `changeValue()` calls and `()` reads have
  nothing to bind to.

Uncommenting those blocks is most of what it would take to build this, which
is a fair measure of how far the two generations had diverged by the time the
file was parked.

## What is on this branch

One file differs from the commit it forks from:
`fw/8bit-quad-fc/8bit-quad-fc.ino`, which is the parked sketch. Everything
else is inherited, so a diff against the parent commit shows the alternative
and nothing else.

The sketch body is kept as found. Only its eight project-local `#include`
lines are repointed at this repository's `src/` layout, the same remap the
baseline sketch got. Two things about it are left alone because they are part
of the artifact:

- It calls `Serial.begin(2000000)` and prints through `Serial`, while the
  `DEBUG()` macros it also uses route to `uart::`. The file was parked in that
  half-converted state.
- It carries no altitude, EEPROM or Mahony code, so several files it inherits
  from the baseline go unused here.

The commit is dated 2018-05-04, the date of the snapshot's own commit, which
is the only date the file can be tied to. The work itself is earlier, since
the fusion in it predates Mahony.

See [`doc/history.md`](doc/history.md) for how the whole lineage was
reconstructed.
