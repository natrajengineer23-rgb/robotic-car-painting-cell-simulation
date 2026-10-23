# Robotic Car Painting Cell — Stage 23

Webots R2025a project featuring two independently rail-mounted UR5e manipulators for near/far vehicle sides and a motor-commanded XYZ overhead gantry for roof, bonnet, boot, and tailgate.

**Status: submission candidate, not industrially validated.** Blue coating, spray coverage and deposition are virtual; collision monitoring uses simplified proxies. The default overhead gantry simulates motor-driven kinematics rather than full physical dynamics.

Open `worlds/CarPaintingCell_Stage23_AllSurfaces_Full.wbt` in Webots R2025a and run; check `README_GANTRY_FIX.md`, `README_STAGE23.md`, and `SUBMISSION_PREFLIGHT.md` before claiming end-to-end validation. This archive includes only the Full world. The report is marked draft. Python offline tests: `python3 -m unittest discover -s tests -v`.
