# poshold_chirp — open items

Cleanup tasks deferred during the poshold_chirp review session. None block flight
testing; pick up before opening an upstream PR.

## Pre-flight verification

- [ ] **Compile for IFLIGHT_F722_TWING with `USE_POSHOLD_CHIRP`** via
      `build_IFLIGHT.sh`. Has not been run since the BF-chirp redesign
      (steps 1–6 of the rotation rework).

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

## Cleanup once design is stable

- [ ] **Drop `posChirpAlignTolerance` from the PG.** No longer used — the
      alignment gate it controlled was removed in step 1 of the redesign.
      Requires a PG version bump.
