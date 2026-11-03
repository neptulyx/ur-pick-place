# Open work for review

Review source: [review/2026-11-03-calibration-collision](https://github.com/neptulyx/ur-pick-place/tree/review/2026-11-03-calibration-collision). Coordinator and Terra reviews are complete.

- Done on this review branch: UR10e arm preview, separate editable tool/camera/mount boxes, cell boxes and saved joint-path review.
- Done in the hand-eye review: original 25/30 degree controls, conservative checks for probes/views/replans/returns, recorded results and runtime checks before each joint command. The default scene blocks automatic motion.
- Done offline: The editor labels tool0 and base box frames and identifies the generic robot model. Firefox reviewed the teacher and calibration editors with an inert service. Open: robot-specific geometry and uncertainty verification, measured cell coverage, tracking/path-fidelity validation and an attended hardware trial. Source and TODO are published on the review branch. The live bench remains unchanged.
- Open in Marker Pick Teacher: integrate actual Cartesian stages and return-to-observation with a suitable path validator. Its new panel is visual only.
- Read [the implementation guide](docs/COLLISION_REVIEW.md).

- Verify the nominal robot model, physical tool, camera, payload and environment geometry against the installed cell. Review contact exclusions, clearances, link uncertainty and the 0.2 degree tracking bound. Native UR tool envelopes and safety planes are not configured. Prepare them as an additional layer; native tool spheres/planes do not solve tool-vs-wrist mesh collision. Keep planning on the Interface PC; no ROS runtime is proposed. See [UR Tool Position](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-usr-man/software/PolyScope/content/safety_g5/Tool_position_g5_en.htm), [UR Safety Planes](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-usr-man/complianceUR16e/H_g5_sections/firstuse/Planes_g5_en.htm), [FCL](https://github.com/flexible-collision-library/fcl), and the reference [anobot revision](https://github.com/khkoh23/anobot/tree/82206d330cf300db9b24aa9f24b2633a92e9db8f). Reference geometry needs measurement and review; do not copy its meshes here.
- Show both staged-motion endpoints before approval; record each stage's target, actual endpoint and status, including partial attempts.
- Make ChArUco test fixtures portable: three tests currently depend on an adjacent generator checkout.
- Keep the teacher minimum Z of 10 mm identified as bench-specific. This review exposes the original hand-eye tilt/roll values. The live bench retains its half-angle defaults.
- Measure pick alignment independently and complete a full gripper pick/release cycle. Commanded-pose following alone cannot establish camera-to-item accuracy.
