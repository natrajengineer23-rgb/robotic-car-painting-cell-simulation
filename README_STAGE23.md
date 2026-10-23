> **Important:** Start with `README_GANTRY_FIX.md`: it supersedes the original physically-dynamic overhead claim. The default overhead uses motor-commanded kinematics, while separate dynamics worlds are experimental.

# Stage 23 — Dual UR5e + motorized overhead gantry for roof, bonnet, boot and rear tailgate

Open `worlds/CarPaintingCell_Stage23_AllSurfaces_Quick.wbt` then Reset/Run. Open `..._Full.wbt` for denser coating. Existing near- and far-side physically powered UR5e controllers and side-panel painting remain intact; new independent overhead Robot is a **3-axis powered XY-Z slider gantry with a servo-tilted nozzle (not a third UR5e)**. The movement is motor actuated and its tip observed using GPS; each patch requires measured pose and position-sensor feedback to pass before virtual blue coating is inserted. Quick=30 new overhead targets; Full=128.

The vehicle is simplified: +X end treated as front bonnet, -X as rear boot/tailgate, raised cab top as roof. Surface setpoints: roof z=1.60 m; bonnet and boot top z=1.13 m; tailgate x=-2.40 m. The nozzle is nominally 0.25 m from these surfaces. Driving motorized overhead gantry moves independently while the two UR5e arms paint sides. Overhead routes retract to safe height before lateral travel and rotate toward the rear tailgate from above. The other robot controllers and radio channels are unchanged.

**Important limitations**: This is **virtual paint geometry**, not sprayed fluids, calibrated paint thickness, a complete automotive body, or a fully-certified robot collision system. The new gantry design is a simulated Cartesian spray manipulator; motor/GPS checks and collision clearances require confirmation in your Webots R2025a runtime. The existing UR5e side controllers do not paint the roof: their low rails cannot reach high roof center. Blue tiles will appear only at independently accepted overhead positions; a failure must stop/skip coating, not fabricate success. The patch grid may leave gaps between tiles, especially Quick mode.  No production safety claims.

Log for overhead: `logs/overhead/stage23_overhead_quick.csv` or `stage23_overhead_full.csv`. Run offline tests using `python3 -m unittest discover -s tests -v` in the extracted project directory. If the overhead controller fails to finish, collect its console output for actuator calibration.

Offline mock dry-run: `python3 tools/offline_overhead_smoke.py`. This uses idealized instant motor response and does NOT prove Webots reachability or collision safety.
