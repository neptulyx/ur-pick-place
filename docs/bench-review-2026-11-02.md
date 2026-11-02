# Bench review: marker teacher, 2026-11-02

This branch contains project-scoped image thresholds and the reviewed two-stage validation changes. The HTML minimum Z default of 10 mm is specific to this bench; it is not a general clearance limit. Full-pose validation preflights endpoints but does not check full geometry or swept paths. The actual intermediate endpoint is not shown before approval and per-stage target/actual/status are not recorded; see [TODO](../TODO.md).

The existing selected offline teaching/validation checks produced 44 passed and 3 skipped; the same three ChArUco cases passed in an isolated layout with the generator as a sibling, for 47 selected checks total. The Node fake wizard passed. These are software/mock checks, not collision or physical validation.

Recorded camera verification passed for 14 fresh observations at replayed poses (maximum residuals 0.796243 mm / 0.182475°). Six completed marker validation attempts showed 0.090444–0.108066 mm and 0.003427–0.004815° feedback error relative to commanded endpoints; three attempts failed and their evidence remains local. This is commanded-pose following, not independent camera-to-item accuracy. Independent alignment measurements and a full gripper cycle remain open.
