# 8bit-quad, abandoned rewrite: cycle scheduler and black box

**This branch is a dead end, kept for the record.** It does not compile and was
never flown. `main` is the working firmware.

Forked from the 2021-03-25 commit and layered onto its working loop, rather
than starting over. Abandoned at some point before work resumed in December
2021, which continued from `main` instead.

## The idea

**Turn the implicit sub-cycle pattern into a real scheduler.** The working
firmware spreads slow work across a four-step counter with a `switch` in
`loop()`; which task lands on which step is a convention held in the author's
head. Here the cycle count is configuration, tasks declare the cycle they run
on, and `cycles::GetBufferIndex()` maps a task's cycle and the current cycle
onto a slot in a ring buffer, so a task reading a sensor two cycles ago gets
the right sample instead of the newest one.

**SD-card black box.** A sized buffer written every two full cycles, with the
budget worked out in a comment: gyro and accelerometer at every sub-cycle,
rate-controller output at every sub-cycle, angle-controller output, RC input,
throttle, altitude and elevation-controller output once per full cycle. The
top-of-file notes also consider streaming the card's contents to the remote,
and benchmarking the card first to use it only if it turns out fast enough.

**Room for navigation.** An empty `namespace nav`, a `TASK_EXTRA` sub-cycle
reserved for "gps/bb", and a `src/portable/` layer with a `requirements.md`
listing what an IMU driver would have to provide.

## Why it stopped

Unknown; there is no note, and this folder's timestamps were destroyed by a
later bulk copy, so only its fork point is known. It was caught mid-edit and
does not compile:

- `cycles.cpp` refers to `cycle`, which does not exist; the parameter is
  called `current_cycle`.
- `cycles.hpp` and `src/tasks/imu/imu.hpp` both use `__IMU_HPP__` as their
  include guard, so whichever is included second is silently skipped.
- `src/portable/v1/imu.hpp` and `src/tasks/imu/imu.cpp` are empty shells whose
  functions do nothing.

## What is on this branch

`fw/8bit-quad-fc/` is the attempt, keeping its own `src/lib/{hw,sw}` layout.
Everything else is inherited from the commit it forks from.

The snapshot re-vendored six libraries inside the sketch that this repository
already carries at the top level: RF24, MPU6050, MPU925x_I2C, the BMP280
driver, MahonyAHRS and UART_atmega328. Those copies are not committed, since
they are the same libraries the build already resolves. Everything the author
wrote is kept, including `src/lib/sw/PID`, which is byte-identical to the
parent's copy, and `src/lib/sw/util`, which is not.

Bench sketches are renamed to the repository's names. The attempt's `IMU_zero`
is dropped as an older copy of the same offset-finder as `calib-imu`. Its two
radio bench sketches are new here and become `fw/tool/rf24-rx` and
`fw/tool/rf24-tx`.

The commit is dated at the fork point, 2021-03-25, because nothing better
survives. The real work happened at some unknown point between then and
December 2021.

See [`doc/history.md`](doc/history.md) for how the whole lineage was
reconstructed.
