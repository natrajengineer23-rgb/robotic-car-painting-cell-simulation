## Hotfix — Dynamic spray geometry parsing

- Fixed an extra closing brace in Webots Cylinder sample geometry in both
  `controllers/stage21_near_arm/surface_paint.py` and
  `controllers/stage21_far_arm/surface_paint.py`.
- Added four-case (two sides × two shape types) VRML brace regression test.
- Re-ran suite: 26/26 offline tests PASS.
- No changes to motion controller, physical rail, pose gates or interlocks.

Stage 21: Two simulated rail-fault injection worlds, a shared-stop propagation CSV auditor, and offline tests. Baseline Stage 20 source otherwise preserved. Webots runtime confirmation required.
Added per-arm post-STOP joint encoder drift traces and fail-closed combined auditor verification.

2026-10-22 Audit usability fix: distinguish full/quick baseline runs from injected-stop evidence, provide actionable messages, strengthen matching of initiator and peer stop reasons, add negative-case tests. Controller and world physics unchanged.


## Stage 23 startup fix
See README_GANTRY_FIX.md for kinematic default, damping experimental variant, encoder guard and shared stop.
