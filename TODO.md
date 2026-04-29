# poshold_chirp — open items

Cleanup tasks deferred during the poshold_chirp review session. None block flight
testing; pick up before opening an upstream PR.

## Code style / cleanup

- [ ] **Indentation in the alignment block.** The new `#ifdef USE_POSHOLD_CHIRP`
      block at the top of `positionControl()` in
      `src/main/flight/autopilot_multirotor.c` uses 2-space indent. Project
      convention (CLAUDE.md, surrounding code) is 4-space. Normalize before
      upstream PR.

- [ ] **Unit comment on yaw rate.** In `src/main/flight/autopilot_multirotor.c`,
      the comment above the `posChirpYawRate = constrainf(...)` line reads
      `// P controller on heading error -> yaw rate (deg), rate limited`.
      The output is in deg/s, not deg. Change `(deg)` to `(deg/s)`.

## Robustness

- [ ] **Apply `yaw_control_reversed`.** GPS rescue handles inverted-yaw setups
      via `rescueYaw *= GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);`
      (see `src/main/flight/gps_rescue_multirotor.c:255`). Mirror the same line
      right after the `constrainf` for `posChirpYawRate`. Without it, users with
      `yaw_control_reversed = ON` will see the drone align to south again.
      `GET_DIRECTION` is in `common/axis.h` (already included);
      `rcControlsConfig()` comes via `fc/rc.h` (already included).
