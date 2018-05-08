# 8bit-quad, abandoned rewrite: external libraries, board "2a"

**This branch is a dead end, kept for the record.** It does not compile and was
never flown. `main` is the working firmware.

A rewrite of the flight controller started 2018-04-10 and dropped 2018-05-09,
onto a new board revision called "2a". It ran in parallel with the firmware
that became this repository's first commit, and outlived it by five days.

## The idea

Stop keeping the drivers and shared types inside the sketch, and pull them out
into independently installed Arduino libraries, so the flight controller, the
remote and any bench sketch could share one definition instead of each
carrying a copy.

- `libraries/MAD-DataTypes` holds the IMU value types and the whole radio
  protocol: packet structs, `SettingId`, and the flight modes.
- `libraries/MAD-Utils` holds the shared helpers.
- The sketch includes them with angle brackets, as installed libraries, rather
  than through a local path.

The protocol it defines is more ambitious than the one on `main`:

- Five flight modes rather than three. Beyond `acro`, `angle` and `direct` it
  adds `horizon`, which follows the stick angle near centre and switches to
  rate control at the extremes, and `land`, an autonomous descent.
- Fixed-point wire fields in centi-units (`throttle_c`, `pitch_c`) instead of
  raw stick counts, so both ends agree on units rather than on scaling.
- A `Control` struct separating the pilot's intent from the packet that
  carries it.
- Settings for calibrating the gyro and accelerometer over the air, which the
  shipped protocol never had.

## Why it stopped

Unknown; there is no note. What is certain is that it was abandoned five days
after the working firmware was published, and that the working firmware kept
its sketch-local drivers for another three years.

It does not build as found. `MAD-DataTypes.hpp` declares `FlyMode::angle` but
two struct initialisers assign `FlyMode::Angle`, which is not a member. That
is preserved rather than fixed, since a compiling version of this code never
existed.

## What is on this branch

`fw/8bit-quad-fc/` is the attempt. Everything else, including the remote
firmware, the custom core and the vendored libraries, is inherited unchanged
from the commit it forks from.

The sketch was `MAD-FC-2a.ino` in a folder named `MAD-FC-2a/code/`, renamed
here to match the repository's layout so that diffing against the parent
commit shows what the rewrite changed. Its sketch-local files keep their
original names, including the inconsistent capitalisation of
`MaD-FC-2a_board.*`.

Its parent, the 2018-05-04 commit, is a slightly later state than what this
was written against: the rewrite forked around 2018-04-10 and the mainline
kept moving until 2018-04-19. No snapshot of that exact state survives.

See [`doc/history.md`](doc/history.md) for how the whole lineage was
reconstructed.
