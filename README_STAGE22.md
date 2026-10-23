# Stage 22 — Webots spray-visualization parser fix (23 October 2026)

This repaired package addresses the actual Webots console error reported from
`CarPaintingCell_Stage21_DualRail_Quick.wbt`, e.g.
`Expected node or PROTO name, found '}'`.

**Cause:** the virtual cylindrical spray mark was formatted using a non-f-string
fragment `subdivision 16 }}`. The extra literal `}` made the dynamic Webots
`importMFNodeFromString` command invalid. The waypoint box tiles were valid,
explaining why a small blue patch appeared while the console continued to report errors.
Both near/far `surface_paint.py` modules now emit a single closing brace.

The previous Stage 21 arm/rail control algorithms, quality gates, interlocks,
and physical motorized rail setup are otherwise unchanged. A regression test
checks brace balance in both geometry types on both sides.

**Open** `worlds/CarPaintingCell_Stage22_Painting_Quick.wbt`, Reset, Run, and check
that the console has no `Expected node or PROTO name, found '}'` messages.
Try `worlds/CarPaintingCell_Stage22_Painting_Full.wbt` for 3 patches on each side.
The two legacy Stage 21 world files are also included and use the same fixed
controllers, though their window titles still say Stage 21.

**Verification:** Offline unit tests pass; Webots R2025a itself is not installed
in this execution environment. Visible paint is a visual proxy, not real fluid
physics or whole-vehicle paint coverage.

---

# Stage 22 — Visible Dual-UR5e Car Painting (Webots R2025a)

**Open** `worlds/CarPaintingCell_Stage22_Painting_Quick.wbt` in Webots for a short demonstration, or `worlds/CarPaintingCell_Stage22_Painting_Full.wbt` for 3 stations and 45 spray targets on each car side. Choose Reset, then Run. Two physical, independent `SliderJoint` rails transport a UR5e each. Actual commanded six-joint motion and pose/spray quality gates are retained from Stage 21.

### What is new

The car starts RED and gets visible glossy BLUE coating patches as the robot's measured nozzle positions and orientation satisfy the existing spray-quality gates. A non-colliding visual blue paint tile is created at each quality-confirmed raster waypoint, supplemented by intermittent quality-gated in-motion sprayed circles. Side ±Y surface locations are used, and all paint events are written to `logs/near/stage22_near_paint_events.csv` or `logs/far/stage22_far_paint_events.csv`.

**Important:** This is a **virtual visual paint overlay**, not real fluid, aerosol CFD, measured film thickness, or proof of complete side, roof, bonnet, or vehicle coverage. Only the pre-existing local raster windows receive visual coating. The upstream Stage 21 quality, coverage, actuator, and link-proxy limitations remain. Industrial safety-rated emergency stops and full-mesh collision protection are NOT demonstrated.

### Files and compatibility

Keep the entire folder structure (`worlds`, `controllers`, `logs`, etc.). The Stage 22 worlds intentionally refer to the **original** `stage21_*` controller names, now upgraded in their folders, to preserve existing interlocks, position tracking, and tests. Unchanged Stage 21 worlds and injection-test worlds are still included as references, and will also use visible painting in normal runs; injected stop test mode must abort before completing the station.

### Run (macOS)

1. Extract the ZIP into Downloads, preserving the top-level folder.
2. Open `worlds/CarPaintingCell_Stage22_Painting_Quick.wbt` in Webots **R2025a** (internet access needed to resolve remote UR5e EXTERNPROTO if not cached).
3. Reset simulation to time 0 and press **Play**. Pan or orbit to either car side. Blue virtual paint patches appear progressively where spraying is accepted.
4. For full 2-rail painting, open `worlds/CarPaintingCell_Stage22_Painting_Full.wbt`, Reset, Play. Inspect console for both rail FINISHED lines and both arm pose/coverage lines.
5. Check `logs/near/stage22_near_paint_events.csv` and `logs/far/stage22_far_paint_events.csv`. Rows identify actual successful world-paint visual inserts. Compare against the existing Stage21 virtual-spray-samples and waypoint logs.
6. For offline regression run (from the project folder): `python3 -m unittest discover -s tests -v`.

### Runtime verification status

Source assembled and unit-regression tested in this sandbox; **Webots itself is not installed in the sandbox**, so actual rendering and controllers in Webots must be verified on your Mac. No real paint-fluid deposition, actual coating uniformity, or verified near/far shared-stop test is claimed.
