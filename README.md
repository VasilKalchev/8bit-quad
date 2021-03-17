# 8bit-quad, abandoned rewrite: portable HAL

**This branch is a dead end, kept for the record.** It does not compile and was
never flown. `main` is the working firmware.

A clean rewrite of the flight controller attempted between 2021-03-15 and
2021-04-03, in parallel with the reorganization that became the 2021 commits
on `main`. Two commits, because it survives as two folders: a bare skeleton,
then a fuller version with the hardware/portable/software split.

## The idea

Two things at once.

**A chip-agnostic hardware layer.** Everything the firmware touches goes
behind a `portable::` namespace - `clk`, `gpio`, `pwm`, `tmr`, `adc`, `rf`,
`pwr` - with the ATmega328 implementation under `src/portable/MAD_2/v100/`
and a `boards.hpp` selecting a board. The intent is that the flight code
stops knowing about AVR registers, so a different MCU means a new folder
rather than a rewrite.

**A declared pipeline.** Every stage in `loop()` is introduced by a doc
comment stating its inputs, outputs, which sub-cycles it runs on, how tight
its timing is, its priority, and which flight modes it applies to. The
working firmware's `loop()` is a long sequence of sections whose ordering
constraints are implicit; this makes them explicit at each stage.

It also sketches a GPS-fed position hold, the only one in the project's
history. It never got further than a stage declaration and a host-side
simulation, `sim/pos_ctrl.cpp`, which works out how to turn a position error
plus the quad's heading into pitch and roll setpoints.

## Why it stopped

Unknown; there is no note. The scope is the likely answer: the portable layer
was never filled in. `adc.hpp`, `gpio.hpp`, `rf.hpp` and the board
`include.hpp` are zero-length files, and the sketch calls into them anyway,
so it cannot build. Meanwhile the mainline reorganization it ran alongside
did reach a working state, and that is what got flown.

## What is on this branch

`fw/8bit-quad-fc/` is the attempt. Everything else is inherited from the
commit it forks from.

- The sketch was `flight_controller.ino`, renamed to match the repository's
  layout so diffs against the parent are readable.
- `sim/` holds two host-side programs, not Arduino sketches: `pos_ctrl.cpp`
  and `sin_cos.cpp`, both `#include <iostream>` and were run on a PC.
- The second commit keeps `FC-alpha2.ino`, the author's reference copy of the
  mainline sketch as it stood around 2021-03-19. It is not part of the build;
  it is the only surviving trace of the mainline between the two commits on
  `main`, which is why it is kept.
- Bench sketches are renamed to the repository's names. The attempt's own
  `IMU_zero` is dropped: it is an older copy of the same offset-finder as
  `calib-imu`.

This forked from the mainline of around 2021-03-18, after the IMU driver work
but before the namespace rename of 2021-03-24. Its parent here is the
2018-05-04 commit, since no commit exists at that intermediate state.

See [`doc/history.md`](doc/history.md) for how the whole lineage was
reconstructed.
