# Stage 22 rendering fix

- Reported runtime symptom: `Expected node or PROTO name, found '}'` on
  dynamically imported spray sample geometry during a successful dual-arm run.
- Root cause: non-interpolated Python literal `subdivision 16 }}` in the
  cylinder geometry string: an extra closing Webots VRML brace.
- Correction: output `subdivision 16 }` in both near and far spray modules.
- Scope: only virtual paint visualization strings and parser regression tests;
  unchanged physical rails, arm motion control, spray gates, interlocks.
- Validation: offline static/unit tests; actual Webots runtime needs local rerun.
- Open: `worlds/CarPaintingCell_Stage22_Painting_Quick.wbt` (or Full).
