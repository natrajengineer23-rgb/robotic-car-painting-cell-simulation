# Stage 23 - Submission preflight (23 October 2026)

## Demonstrated in candidate's supplied Webots console
- Near-side UR5e: 3/3 stations, 45/45 side raster poses.
- Far-side UR5e: 3/3 stations, 45/45 side raster poses.
- Overhead gantry homing: `GANTRY_HOME_VERIFIED`.
- Overhead roof: 48/48 qualified; bonnet: 36/36; boot: 24/24.
- Multiple screenshots show blue virtual coating on sides and top surfaces.

## Still needed before calling the assessment *complete*
1. **Run the revised Full world from this package in Webots**. Both default and experimental Stage 23 worlds had a duplicate `dampingConstant` declaration; all four are corrected here. Do not claim the static check validates real Webots runtime.
2. **Confirm rear tailgate + final completion:** `PANEL tailgate : qualified 20 / 20` and `FINISHED: roof=48/48 hood=36/36 boot=24/24 tailgate=20/20`; preserve full console and `logs/overhead/stage23_overhead_full.csv`. The candidate's pasted log ended after `PANEL boot : qualified 24 / 24`.
3. **Test 3-7 minute Webots demonstration video**. Provided console had libx264 two-pass bitrate error (`2pass curve failed to converge`) despite `Video creation finished`. Play the video all the way through, and re-encode at a lower bitrate if needed.
4. **Finalize report**: `Technical_Report_Stage23_DRAFT.pdf` is four pages, but remains a draft until tailgate is observed and the results updated. The older `CarPaintingCell_Submission_Kit/Technical_Report.pdf` describes Stage 6 and is not suitable as a Stage 23 report.
5. **Public GitHub**: push runnable source and clear README; verify repo in an incognito browser.
6. **Google Drive**: provide `Technical_Report.pdf`, `Simulation_Files/`, `Screenshots/`, `Demo_Video/` and `GitHub_Link`; set General Access -> Anyone with link -> Viewer. Verify permission.
7. **Final email**: to connect@yuktiiai.in with subject `Assessment submission on robotics simulation`, including full name, college, email, simulator, robot(s), public GitHub link and Drive link.

## What this local package includes
- Ten Webots worlds; the new full/quick default `CarPaintingCell_Stage23_AllSurfaces_*.wbt` worlds are recommended.
- All controller Python source, test suite, offline analysis scripts, source README files.
- Four-page updated report DRAFT and three candidate-run screenshots plus cropped viewing copies.
- This preflight note and regression test for the parser defect.
- No recorded demo video or verified public GitHub / Drive URLs; these are not created or implied here.

## Engineering boundaries
- Blue coating and deposition quality are *virtual/synthetic*; there are no paint-fluid dynamics.
- The two UR5e rails are motorized in Webots. The default overhead gantry has actuator-driven kinematics without dynamic moving-part Physics nodes. Experimental dynamics world is not certified/validated.
- Proxy clearances and basic AABB checks do not prove full collision safety.
