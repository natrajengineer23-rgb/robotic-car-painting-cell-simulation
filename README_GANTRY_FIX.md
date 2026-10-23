# Stage 23 overhead gantry startup-stability fix

The prior Webots full-world run failed on the overhead mechanism during `home` with Y encoder ~0.5754 m and Z encoder ~3.17345 m, exceeding the 1.92 m Z motor limit. Its dynamic joint hierarchy started exactly on lower physical stops, with unlimited acceleration by default and no damping.

## Worlds to open
- `worlds/CarPaintingCell_Stage23_AllSurfaces_Quick.wbt` — **recommended first**: motor-commanded, kinematic overhead mechanism; the two existing UR5e rail systems retain their original physics and painting controllers.
- `worlds/CarPaintingCell_Stage23_AllSurfaces_Full.wbt` — the same robust overhead mechanism with 128 extra virtual paint targets.
- `worlds/CarPaintingCell_Stage23_GantryDynamics_Experimental_Quick.wbt` and `_Full.wbt` — physics-enabled overhead variant with hard-stop margin, constrained acceleration and extra damping; **not Webots-validated**.

**Important engineering distinction:** The two standard Stage 23 worlds use actuated SliderJoint/LinearMotor and HingeJoint/RotationalMotor with motor encoders and tip GPS, **but the new gantry moving parts have no Physics nodes**. Thus overhead movement is motor-driven **kinematics simulation**, not physical force/torque dynamics. No Supervisor relocation/teleportation is used. Side rails retain their original physical motor setups. Do not describe the overhead gantry as dynamics-validated or collision-certified.

Other fixes: enlarged per-waypoint timeout for long motor traversals; checks every encoder against joint hard limits; verifies homing; STOP broadcasts on existing cell channel 33 from the overhead controller upon failure, so both side rail controllers can abort rather than continuing independently; safe clamp when commanded to hold position after failure. The planner itself and all four paint surface target sets are unchanged. Existing measured GPS + encoder + surface-gap paint gate remains.

## Run and verify
1. Close the previous project and extract this archive as a **new folder**. Open the new `...AllSurfaces_Quick.wbt`, Reset then Play in Webots R2025a. Do not reuse a running Stage 23 world from the old folder.
2. Check that console prints `GANTRY_HOME_VERIFIED`, then `PANEL roof : qualified 9 / 9`, `PANEL hood : qualified 9 / 9`, `PANEL boot : qualified 6 / 6`, `PANEL tailgate : qualified 6 / 6` and `FINISHED`. If a gate fails, inspect `logs/overhead/stage23_overhead_quick.csv`.
3. Run `_Full.wbt` for 48 roof + 36 bonnet + 24 boot + 20 tailgate targets. Existing side arms target 45 each.
4. Verify the stage doesn't hit `GANTRY_ENCODER_OUT_OF_BOUNDS`, `GANTRY_MOTION_TIMEOUT` or `STOP/FAILED` before trusting results.

## Validation statement
Offline source syntax and unit tests are checked; controller is exercised in a mocked actuator test. This environment has **no Webots process**. The actual motor motion, GPS tracking, visual deposition and interlocks require a user-side Webots run. None of the supplied worlds guarantee full collision or paint physics.
