## Follow-up verification — Webots VRML parser fix

After the user's Webots console reported `Expected node or PROTO name, found '}'`,
the dynamic `Cylinder` string in BOTH side-paint modules was corrected to use
one closing brace. The new `test_emitted_geometry_is_balanced_vrml` protects
both geometry types and both sides against this regression. Offline unittest
suite: **26 tests passed**. See `verification/stage22_parser_fix_26_tests.log`.

No Webots runtime replay was possible in this environment. Prior Webots
runtime output (provided by the user) demonstrates the arm raster trajectories
completed but the previous spray-circle imports were rejected.

---

# Stage 21 — Automated validation evidence

Date: 22 October 2026

## Executed successfully in the independent Linux test environment

- 20 Python offline tests passed (`python3 -m unittest discover -s tests -v`).
- Both directions of shared-stop propagation were tested by executing the **actual, unchanged** `stage21_far_rail.py` and `stage21_near_rail.py` controller programs through a deterministic mock of Webots' `Robot`, slider motor, encoder, GPS, and radio APIs.
- Far-side injection caused the mock far rail to record `INJECTED_TEST_CELL_STOP_FROM_FAR` and the mock near rail to record `SHARED_CELL_INTERLOCK CELL_STOP:far:INJECTED_TEST_CELL_STOP_FROM_FAR`. Both emitted a STOP to their respective mocked arm radio channels.
- Near-side injection produced the corresponding reverse results.
- Both integration tests verified that neither mock rail reported a completed painting station and that peer rail-stop timestamps differ from the initiator by no more than 0.100 simulated seconds in this mock scheduler.
- All Python source files in `controllers`, `tools`, and `tests` parsed without syntax errors.

## NOT performed / cannot be claimed as validated

- No real Webots R2025a process was available in this execution environment, and there is no access to the user's running macOS Webots application.
- Thus the real injected-stop Quick `.wbt` worlds have **NOT** been run in Webots here, the actual UR5e articulated arm motion and post-STOP joint encoder hold have **NOT** been tested here, and `tools/audit_cell_interlock.py` has **NOT** been supplied with actual runtime fault-injection logs.
- The test stubs instantaneously follow rail position commands, do not model physics or radio transmission latency, and simulate ARM_READY/STARTED handshakes rather than executing the articulated arm controller.
- These tests do not establish industrial safety performance, real braking distances, complete collision clearance, or actual paint deposition.

## Reproducibility

Run `python3 -m unittest discover -s tests -v` at the extracted project root. The added `tests/test_stage21_rail_protocol_integration.py` uses only Python's standard library; it does not change either Webots world or any production controller and uses temporary log folders.

**Required remaining acceptance evidence:** two live Webots runs, `CarPaintingCell_Stage21_InjectStop_Far.wbt` and `CarPaintingCell_Stage21_InjectStop_Near.wbt`; inspect fault logs and run `python3 tools/audit_cell_interlock.py --injected-side far` or `near` after the corresponding run. Expected full audit includes the actual 1.024s post-stop joint hold diagnostic for both arms.
