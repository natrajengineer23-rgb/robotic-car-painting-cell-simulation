# Stage 21 Webots validation checklist

Tick these **only after observing the user-run Webots R2025a output**. No claim below is prevalidated by the packaged source or offline tests.

## Quick world (one station per side)
- [ ] `stage21_near_arm` and `stage21_far_arm` both report `PROXY_GUARD_ACTIVE`.
- [ ] Each physical rail receives `ARM_READY` and reports `PREFLIGHT PASSED`.
- [ ] 15 `POSE_PASS` outputs from each arm; 15/15 each side and 30/30 combined.
- [ ] Each raster patch reports nonzero quality-gated relative dose samples and finite CV.
- [ ] No `SAFETY STOP` or `GUARD STOP` in either console.
- [ ] Run `python3 tools/analyze_dual_quality.py --mode quick` from project root; valid report written.

## Full world (three stations per side)
- [ ] Each arm reports 45 `POSE_PASS` results, across three stations.
- [ ] Each rail reports 3/3 and motor holds during arm tracking.
- [ ] Six separate relative-deposition summaries exist across near/far sides.
- [ ] Each side's virtual-spray log has nonzero gated samples per patch.
- [ ] No unexpected arm, joint-proxy, rail-drift or shared-cell stops.
- [ ] Run `python3 tools/analyze_dual_quality.py --mode full` and confirm 90/90 and consistency checks.
- [ ] Run `python3 tools/render_dual_heatmaps.py` and inspect all six maps using the **shared** color scale.
- [ ] Compare CV and apron dose *proxies* for each corresponding near/far patch; note if asymmetry emerges.
- [ ] Save console screenshots plus generated logs; don't publish simulated dose as coating thickness.

## Unresolved engineering work
- [ ] Independently test cross-side `CELL_STOP` fault propagation with a synthetic fault injection.
- [ ] Independently benchmark a sequential single-rail painting run under equal runtime conditions.
- [ ] Calibrate the assumed Gaussian profile and stand-off scaling to a measured spray model (future work).
- [ ] Validate full robot-link collision geometry and swept volumes, not just centerline proxies.
- [ ] Plan roof, bonnet, boot, front/rear and nonplanar panel coverage.
- [ ] Evaluate ventilation, paint delivery/overspray interaction, personnel exclusion and safety-rated E-stop before any physical cell.
