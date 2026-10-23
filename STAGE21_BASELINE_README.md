# Stage 21 — Dual-Rail Shared Software-Stop Propagation Test

STATUS: SOURCE CONSTRUCTED AND OFFLINE TESTED. REAL WEBOTS RUNTIME NOT YET TESTED.

This is an **additive fault-injection safety experiment** built from the Stage 20 dual-rail concurrent painting and assumed relative-deposition model, preserving the original independent powered rails, 6-DOF coupled pose control, deposition quality gating, and active simulated link-proxy guards. Stage 20 is intentionally unchanged.

## Why Stage 21

Stage 20 has per-rail rail/arm stop propagation and broadcasts failures over software channel 33. Having a broadcast in the source code is not proof that the **opposite-side rail and robot** will stop during a fault. Stage 21 introduces two separate worlds to verify the stop propagation in Webots. This is software-only *simulated fault injection* and must never be presented as an industrial safety-rated stop or verified collision protection.

## Run, in order

1. Keep the extracted package directory intact under Downloads, with `controllers`, `worlds`, `tools`, and `logs` siblings.
2. Open `worlds/CarPaintingCell_Stage21_DualRail_Quick.wbt`; Reset > Play. Expected unchanged baseline: 15/15 poses for BOTH arms and no stops.
3. **Do not run the interlock auditor yet.** The normal FULL world writes three PASS station rows, which the auditor correctly rejects. Open `worlds/CarPaintingCell_Stage21_InjectStop_Far.wbt`; Reset > Play. Once far-side rail gets `ARM STARTED`, a **synthetic rail controller fault** triggers after 4 simulated seconds, sends STOP to far arm and broadcasts on shared channel 33. Near rail should report `SHARED_CELL_INTERLOCK` and tell near arm STOP; no raster station should report completion. Continue simulation until BOTH rail logs have reported `SAFETY STOP` and BOTH arms have printed `STOP HOLD DIAGNOSTIC`, then pause Webots. Both arms independently log 1.024s joint-hold diagnostics in `logs/<side>/stage21_<side>_arm_stop.csv`.
4. Run in Terminal (working directory must be the Stage 21 project): `python3 tools/audit_cell_interlock.py --injected-side far`.
5. Open `worlds/CarPaintingCell_Stage21_InjectStop_Near.wbt`; Reset > Play until the STOP and joint-hold diagnostics have been printed, then pause Webots and run `python3 tools/audit_cell_interlock.py --injected-side near`.
6. Optionally run `worlds/CarPaintingCell_Stage21_DualRail_Full.wbt` (45/45 per arm) and `python3 tools/analyze_dual_quality.py --mode full` to check for regressions.
7. `python3 -m unittest discover -s tests -v` runs all available static/unit regression checks. NOT a Webots test.

**Important**: The generated CSVs are overwritten whenever a new world is run. An error such as `Expected exactly one aborted station on far, got 3` means you audited NORMAL FULL (three PASS stations), not the injected-stop QUICK world. It does NOT mean a simulated shared-stop failure occurred. The new auditor prints the correct world and next action. Audit each injected test immediately after it completes and save its output if you need independent evidence. An expected injected stop should NOT count as successful painting. The stopper and opposite rail CSV must both record fault statuses. If not, Stage 21 fault validation fails even if Webots continues playing.

## Further limitations

This tests software message propagation, both rail stop records, and 1.024s of joint-encoder hold telemetry. It does not verify hardware E-stop, PL/SIL, measured industrial stopping distance, or complete collision checking. The Stage 20 simulated deposition model is also uncalibrated. Proper fail-safe recovery/interlock latching, motion-verified stopping of both arms, real paint flow, roof/bonnet and end surfaces remain future work.

## Stage 21 audit usability revision (22 October 2026)

- No painting-control algorithm or original Webots world has been modified.
- `tools/audit_cell_interlock.py` now specifically recognizes normal FULL (three PASS stations) and Quick (one PASS station) records and prints the appropriate recovery step.
- Added tests for both misuse cases; injected fault validation still requires real Webots runtime records from both rails and both arms.
