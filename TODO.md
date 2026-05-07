# poshold_chirp — open items

Cleanup tasks deferred during the poshold_chirp review session. None block flight
testing; pick up before opening an upstream PR.

## Pre-flight verification

- [ ] **Bench-test the centi-Hz fix** (PG v5, 2026-05-05). Rebuild for
      IFLIGHT_F722_TWING with `USE_POSHOLD_CHIRP` via `build_IFLIGHT.sh`,
      reflash (PG version bumped from 4 → 5, so defaults reload), engage
      POSHOLD then POSHOLD_CHIRP. After the 1s settle, confirm
      `DEBUG_POSHOLD_CHIRP` slot 3 (`fchirp`) sweeps from ~0.05 Hz toward
      end freq, slot 2 (`posChirpAmpl·exc`) is a sweeping sinusoid, and
      slot 1 (BF angle on chirped axis) follows it.

- [ ] **Investigate GPS rate fluctuation.** User observed the GPS data rate
      is not stable in flight (2026-05-05). The chirp time mapping assumes
      a stable rate (passed once to `chirpInit` at mode entry); fluctuation
      will warp `fchirp` vs. wall-clock. Decide whether to (a) re-init the
      chirp on rate change, (b) document the inaccuracy, or (c) gate chirp
      start on a stability check.

## Code style / cleanup

- [ ] **Indentation in the `#ifdef USE_POSHOLD_CHIRP` block at the top of
      `positionControl()`** in `src/main/flight/autopilot_multirotor.c`
      (around lines 262–298) uses 2-space indent. Project convention
      (CLAUDE.md, surrounding code) is 4-space. Normalize before upstream PR.

- [ ] **Indentation in the PID-reset block** (around lines 311–321 of
      `src/main/flight/autopilot_multirotor.c`) is mixed 6/8-space. Normalize
      to 4-space.

- [ ] **Stale comment, line ~281** of `src/main/flight/autopilot_multirotor.c`:
      `// shortes-path heading error to captured target , wrapped to (180, -180]`
      has a typo (`shortes-path`, doubled space, reversed range notation).
      Fix to `// shortest-path heading error to captured target, wrapped to (-180, 180]`.

- [ ] **Stale comment, line ~464** of `src/main/flight/autopilot_multirotor.c`:
      `// Because the drone aligns to North, LON (East/West) maps to ROLL,
      and LAT (North/South) maps to PITCH.` is no longer true under the BF-chirp
      redesign. The chirp is generated in BF and pre-rotated to EF; the existing
      downstream EF→BF rotation produces pure-axis BF commands at any heading.
      Rewrite to reflect the new design.

## Robustness

- [ ] **Apply `yaw_control_reversed`.** GPS rescue handles inverted-yaw setups
      via `rescueYaw *= GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);`
      (see `src/main/flight/gps_rescue_multirotor.c:255`). Mirror the same line
      right after the `constrainf` for `posChirpYawRate`. Without it, users with
      `yaw_control_reversed = ON` will see the drone rotate opposite to the
      heading-hold P controller's intent.
      `GET_DIRECTION` is in `common/axis.h` (already included);
      `rcControlsConfig()` comes via `fc/rc.h` (already included).

- [ ] **Consider hard-blocking chirp start when `!compassIsHealthy()`.**
      Currently mag failure produces only the `PCHRP NO MAG` OSD warning. With
      the BF-chirp redesign, heading is now load-bearing for the EF injection
      rotation (not just for alignment), so a bad mag silently corrupts the
      sysID data. Promote to a hard gate on chirp init, or at minimum document
      the risk.

