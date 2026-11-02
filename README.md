# ur-pick-place

Teach waypoints and run force-guided pick-and-place cycles on a Universal Robots
arm with an OnRobot RG2 or Arduino gripper. An optional camera UI teaches a pick
position relative to an ArUco marker or ChArUco board.

| UI | Start command | Address | Use it to |
| --- | --- | --- | --- |
| Robot Bench | `./start.sh` | http://127.0.0.1:8765 | Record waypoints and run a pick-and-place cycle |
| Marker Pick Teacher | `./start_marker_teach.sh` | http://127.0.0.1:8766 | Teach, export and validate camera-derived pick positions |

## 1. Try Robot Bench without hardware

From this repository, run:

```sh
./start.sh --demo
```

The launcher creates `.venv`, installs dependencies and opens the browser. Python
3.12 is recommended; Python needs `venv` support. Keep the terminal running.

1. Click **Connect RTDE** to connect to the simulated robot.
2. Open **Joint & force tests** at the top of the page.
3. Click **Load demo poses**.
4. Click **Preview cycle** and review the summary.
5. Click **Run one cycle** and watch the six progress stages.

Demo mode simulates the robot and gripper. Its saved poses are separate from live
poses and cannot be used for live motion.

## 2. Connect a real robot

Stop the demo with `Ctrl+C` in its terminal, then run:

```sh
./start.sh
```

1. On the robot, configure the active gripper TCP, payload and remote control.
   Disconnect other applications that control the robot through RTDE.
2. In Robot Bench, enter the **Robot IP**.
3. While disconnected, expand **Motion limits & workspace**, set speeds and XYZ
   bounds for your cell, then click **Save settings**. The bounds must include the
   current TCP and the intended targets.
4. Click **Connect RTDE**. Check that live TCP and joint readings appear.

Start at reduced speed with the physical emergency stop available. Motion checks
cover TCP workspace limits and robot safety/reachability; they do not detect
obstacles or check the whole arm's path. Hardware operation still requires
validation on your cell; automated tests use simulated hardware.

## 3. Record Home, Pick and Place

On **Record waypoints**, repeat these steps for each position:

1. Position the robot with the pendant or **Jog the TCP** controls. Hold a direction
   button to move; release it to stop. Position and rotation use robot-base axes.
2. Wait for the robot to stop.
3. Enter a **Pose name** such as `home`, `pick` or `place`.
4. Click **Record current pose**. The saved card contains both TCP coordinates
   and joint angles.

Use a clear transit position for Home. Teach Pick as the lowest intended search
endpoint, with tool Z within 20° of base −Z. Place sets the placement XY,
orientation and reference height; release occurs at the detected contact pose.

On a saved pose card:

- **Load target** fills the target fields without moving.
- **Move TCP** moves linearly to the saved TCP pose.
- **Move joints** moves to the saved joint angles.
- **Update from current** replaces the pose with the stopped robot's current pose.
- **Rename** changes its name.

For an absolute move, click **Use current**, edit **TCP target**, then click
**Move to TCP target**. XYZ uses millimetres; Rx/Ry/Rz are rotation-vector
components in degrees, not roll/pitch/yaw.

## 4. Preview and run a pick-and-place cycle

1. Open **Joint & force tests**. To test a waypoint alone, choose a **Joint
   waypoint** and click **Test joint move**; this does not operate the gripper.
2. In **Pick & place setup**, select **Home**, **Pick** and **Place**. Home uses
   saved joint angles; Pick and Place use saved TCP poses.
3. Choose **Gripper type** and configure it:
   - **OnRobot RG2:** open width, grip width and grip force.
   - **Arduino gripper:** serial port, close command and calibrated grip/release
     thresholds for both pressure pads. These are raw readings, not newtons.
4. Expand **Force, approach & grasp offset**. Review approach heights, forces,
   descent limits/speeds, earlier-contact backoff, grasp offset, lift and retract.
   Backoff and XYZ offsets change where the gripper closes after contact; set
   them to zero if you intend to grip at the contact pose.
5. Click **Preview cycle** and review the selected poses and settings.
6. With the path clear, click **Run one cycle**.

The cycle goes through Home/open → pick approach → force search → offset/grip →
verify/lift/Home → force-place/retract. Pick descent stops at contact or its taught
endpoint; placement requires contact before release.

Keep the tab active during movement. **Stop motion** cancels the sequence without
opening the gripper or moving to a recovery position. A lost UI heartbeat also
cancels it. This is a software stop; use the robot's physical emergency stop when
needed. Inspect the robot and gripper before restarting.

Each run uses one preview. Preview again before another run and after changing
settings or waypoints. Manual movement and pose recording are blocked during a cycle.

## 5. Teach a camera-derived pick (optional)

You need a calibrated wrist camera, its `calibration.json` from `ur10e-handeye`,
and a measured ArUco marker or ChArUco board fixed to the object. Keep the camera
mounting and camera profile unchanged from calibration. Vision needs Python 3.11–3.13
(3.12 recommended); live capture also needs `charuco-generator` and
`charuco-realsense`. The launcher installs those sibling repositories if present.

```sh
./start_marker_teach.sh
```

Open http://127.0.0.1:8766. Follow the five steps with **Next** and **Back**:

1. **Set up:** choose the calibration, robot IP and marker type. For ChArUco,
   select the **Printed picking board** attached to your object, or enter its
   manifest and board name. The calibration board is not needed in view during
   picking. For ArUco, enter dictionary, ID and
   measured outer black-square size, excluding the white margin. Copy the active
   TCP offset from PolyScope **Installation → TCP**: XYZ in millimetres and
   RX/RY/RZ in rotation-vector radians. It must match the calibration TCP.
   Confirm the checkbox, then click **Check setup & continue →**.
   Under **Project & advanced settings**, choose a project folder, approach height
   (default 100 mm) and movement limits. **Full 3D pose** is the default: follow
   XYZ and orientation when AGV parking changes position, height, roll, pitch or
   yaw. **XY + yaw** is an optional mode that preserves the taught height and tilt.
2. **Capture marker:** use the pendant to make the whole target visible, stop the
   arm, then click **Capture marker**. Review the image and detected axes.
3. **Teach pick:** keep the marker and object fixed. Pendant-jog the gripper to
   the desired final pick endpoint and orientation, then stop. Confirm that the
   marker has not moved and click **Read current TCP & save pick**. Check that
   the joint-motion path is clear, then click **Return to observation pose**.
   Return speed and workspace bounds are under Step 5’s **Test motion settings**.
4. **Read & export:** show the whole marker and stop the arm. The capture pose may
   differ from teaching; returning to the saved observation joints is optional.
   Click **Read marker & export poses**. Download **Pick pose JSON**
   (`target.json`); the approach is above the pick along robot-base +Z.
5. **Validation:** reposition the marker and object together within the allowed
   movement range. Set workspace bounds and speeds under **Test motion settings**.
   From any clear camera view, stop the arm and click **Capture for validation**.
   Review the image and all six target coordinates. Confirm that the AGV and
   marker stayed fixed and both paths are clear, then click **Move to full
   3D pick**. Full projects move to pick XY and orientation at the current height,
   then descend vertically to the pick height. They follow the calculated XYZ and orientation,
   including height and tilt changes. The tool can descend to the taught endpoint;
   this test has no contact search and does not operate the gripper. Planar
   projects retain **Move at fixed Z**, which holds the captured TCP height.
   The approach height setting applies to exports, not this test. Keep the tab active. After
   the move, visually check alignment and choose **Alignment looks correct** or
   **Alignment needs adjustment**.

Return moves use saved joint angles and can change TCP Z; the default return
speed is 3°/s. Check clearance before clicking either return button.

Each validation attempt needs a new capture less than 60 seconds old. Step 5 turns
green only after a completed move is marked visually aligned. TCP arrival checks
alone do not establish camera accuracy. **Stop motion** stays in the header.

To resume teaching, enter the project folder under **Project & advanced settings**
and click **Open saved project**, then recheck the active TCP. Capturing a new
teaching reference requires teaching the pick again.

For an AGV test, teach once, then vary X, Y, Z, roll, pitch and yaw individually
and together within the configured limits. Repark the AGV or reposition the
marker and object rigidly together; keep the AGV stopped throughout each capture
and movement. Use a fresh Step 5 capture at each position without replacing the
Step 2 reference. Measure physical XYZ and orientation alignment independently
of the robot's commanded-pose arrival check. Defaults still bound target shift
to 100 mm, marker tilt change to 10° and tool tilt to 20° from base −Z.
Full-pose calculations compensate Z; the planar 10 mm height-change tolerance
does not apply in full mode. Step 5 uses the current camera capture; automatic
camera-view refinement is tested through Robot Bench's live picking workflow.

## 6. Use the camera pick in Robot Bench

For a fresh measurement on each run:

1. In Marker Pick Teacher, select the camera and enable **Refine before picking**
   when creating a new project. Capture a clear teaching view at about 300 mm,
   then teach and save the pick as usual.
2. In Robot Bench, open **Joint & force tests → Camera target**. Enter the saved
   project's full `vision.json` path in **Live vision configuration**. Select
   Home/Place and configure the gripper, approach height and force limits.
3. Connect, keep the whole pick marker visible and stationary, then click
   **Preview cycle**. This captures without moving the robot.
4. Review the preview and clear the refinement and picking paths. **Run one cycle**
   captures again, makes one bounded move to the taught camera-to-marker view,
   settles, captures again and checks that view before picking. The initial arm
   joints may differ from teaching. A failed refinement blocks the pick.

The external system must stop the AGV before arm motion and keep it stopped
throughout capture and picking. No AGV controller or interlock is added here.
Existing workspace, reachability, target-shift and preview-change limits remain
active. A moved AGV needs a fresh preview. Place poses saved in the robot base
frame must still be valid at the parked location.

Refinement defaults to a 5 mm / 2° view-match tolerance, configurable through
`pick.refine.max_view_error_mm` and `max_view_error_deg` in `vision.json`. These
check reproduction of the camera view; they do not certify ±2 mm picking accuracy.
Detection thresholds and accepted-frame averaging are unchanged. When refinement
is disabled, localization uses the calibrated transform directly without a
same-joint-angle restriction.

For manual snapshot transfer:

1. In Marker Pick Teacher, re-read the marker and download a fresh `target.json`.
2. In Robot Bench, open **Joint & force tests → Camera target** and load that file.
   It overrides the saved Pick selection.
3. Set **Pick approach** to the teacher's approach height (100 mm by default).
4. Select your taught Home/Place and gripper settings, then **Preview cycle**.
5. Review the path and settings, then click **Run one cycle**.

Exported targets are static snapshots: re-read and reload for each new pick.
Camera target JSON uses `[x, y, z, rx, ry, rz]` in the robot base frame,
**metres and rotation-vector radians**.

### Basler a2A2448-23gmBAS

Install the additional adapter with `.venv/bin/python -m pip install -e '.[basler]'`.
Choose **Basler** in Marker Pick Teacher and select its own hand-eye calibration.
The calibration must describe this camera's serial, lens intrinsics/distortion,
2448 × 2048 image profile, mounting and active TCP. RealSense calibration and its
taught grasp cannot be reused after the camera swap.

[examples/vision-basler.json](examples/vision-basler.json) illustrates the profile
and optional manual exposure (`exposure_time_us`). Set exposure for the actual
lighting; focus and lock the 16 mm lens before calibrating. The adapter uses Mono8,
software-triggered images and the calibrated lens model; it resets crop offsets,
binning and decimation to preserve the image geometry. Camera SDK imports are lazy,
so RealSense operation does not require pypylon.

### Check the ±2 mm application requirement

Test across the expected parking variations and record independently measured
TCP error at the actual pick point, including Z. Step 5 accepts optional measured
XYZ errors and stores their magnitude and whether they are within 2 mm. Its
fixed-Z visual test and controller arrival error alone do not prove pick accuracy.

The vision report includes frame spread, marker-to-pick distance and the rotational
contribution to pick-point spread. At a 300 mm offset, 0.2° angular error can cause
about 1.05 mm displacement. Frame spread measures repeatability, not systematic
calibration error. Robot Bench saves the last completed live plan alongside its
sequence settings as `latest_vision_pick.json`.

## Saved files and useful commands

Robot Bench saves poses automatically in `data/live/poses.json` (demo:
`data/demo/poses.json`). **Export JSON** downloads the pose library. Marker
teaching projects default to `data/marker-teaching/`; keep their files together
and retain access to the referenced calibration and board manifest.

```sh
./start.sh --config pick_place.json  # Load recipe defaults into Robot Bench
./start.sh --no-browser --port 8765  # Open the browser address manually
./start_marker_teach.sh --port 8767 # Use another port
./start.sh test                     # Run offline tests
./start.sh pick-place --help        # Pick-and-place CLI options
./start.sh vision --help            # Camera CLI options
```

Recipe examples: [RG2](examples/rg2.json), [Arduino](examples/arduino.json),
[ArUco vision](examples/vision-aruco.json), [ChArUco vision](examples/vision-charuco.json).
Disconnect Robot Bench before running a CLI job that needs RTDE control.
