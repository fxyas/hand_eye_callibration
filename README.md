# Hand-eye calibration for ROS 2 (`universal_handeye`)

A ROS 2 package that finds the fixed transform between a robot arm and a camera. It uses OpenCV
`calibrateHandEye()` with poses read from TF and a chessboard or ArUco target seen by the camera.
Then it uses that transform to send the arm to an ArUco marker it sees.

- **Eye-to-hand**: the camera is fixed (on the robot body, a stand, the ceiling) and the target is on the gripper.
  Result: `base_frame → camera_frame`.
- **Eye-in-hand**: the camera is on the gripper and the target is fixed in the world.
  Result: `tool_frame → camera_frame`.

Targets: chessboard or a single ArUco marker. Solvers: Tsai, Park, Horaud, Andreff, Daniilidis.
Tested on ROS 2 Humble with a ZED camera (`eye_to_hand`, ArUco `DICT_5X5_50`, `base_link` → `right_tcp`).

```
src/universal_handeye/
├── config/handeye.yaml                 calibration settings
├── config/goal_publisher.yaml          ArUco goal publisher settings
├── launch/handeye_calibration.launch.py
├── launch/aruco_goal_publisher.launch.py
├── universal_handeye/                  nodes and modules (see "How it works")
└── scripts/aruco_pub.py                standalone USB-camera ArUco → TF demo
```

## 1. Install

Requirements: ROS 2 Humble, a calibrated camera that publishes `sensor_msgs/Image` and `sensor_msgs/CameraInfo`,
and a robot that publishes TF from its base to its tool (for example through `robot_state_publisher`).

```bash
sudo apt install ros-humble-cv-bridge ros-humble-tf-transformations \
                 python3-opencv python3-scipy python3-yaml

git clone https://github.com/fxyas/hand_eye_callibration.git hand_eye_ws
cd hand_eye_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. Prepare the target

| Target | What to print | Config |
|---|---|---|
| ArUco | One marker from the configured dictionary (default `DICT_5X5_50`). Measure the black square edge after printing | `target.type: aruco`, `target.marker_size` (m), `target.dictionary`, `target.marker_id` (`-1` = any) |
| Chessboard | A flat board. `rows` × `columns` counts the **inner** corners | `target.type: chessboard`, `target.rows`, `target.columns`, `target.square_size` (m) |

Mount it rigidly: on the gripper for eye-to-hand, or fixed in the scene for eye-in-hand. A marker that wobbles
or a wrong `marker_size` shows up directly as calibration error.

## 3. Configure

Edit `src/universal_handeye/config/handeye.yaml`, or copy it and pass it with `config_file:=`.

```yaml
universal_handeye:
  ros__parameters:
    mode: eye_to_hand                 # or eye_in_hand
    camera:
      image_topic: /zed/zed_node/left/color/rect/image
      camera_info_topic: /zed/zed_node/left/color/rect/camera_info
      frame: camera_link              # child frame of the published result (see note below)
    display:
      show_image: true                # needed for the keyboard controls
    output:
      file_path: "hand_eye_calibration_output.yaml"   # relative to the directory you launch from
    robot:
      base_frame: base_link
      tool_frame: right_tcp
    target:
      type: aruco
      marker_size: 0.05
      dictionary: DICT_5X5_50
      marker_id: -1
    solver:
      method: TSAI
    samples:
      required: 20                    # samples before (C)alibrate is allowed (the solver needs at least 5)
      min_translation: 0.05           # m: a new sample must be this far from every earlier sample...
      min_rotation: 10.0              # deg: ...and rotated at least this much
```

Every key must stay under `ros__parameters`. If a key ends up outside it, ROS fails to load the file or silently
uses the default. The code defaults are `samples.min_translation: 0.1`, `samples.min_rotation: 15.0` and
`target.dictionary: DICT_4X4_50`.

**About `camera.frame`.** OpenCV gives the target pose in the camera's **optical** frame (z forward, x right,
y down). The result is therefore `base → optical frame`, published under the name you put in `camera.frame`. Use
either the camera's optical frame name, or a new name that nothing else publishes. If the camera driver already
publishes that frame with another parent, TF gets two parents for it and lookups break.

## 4. Calibrate

```bash
ros2 launch universal_handeye handeye_calibration.launch.py
# or: ros2 launch universal_handeye handeye_calibration.launch.py config_file:=/path/to/my_handeye.yaml
```

A window called **HandEye Camera** opens with the detected target drawn on it (axes for ArUco, corners for a
chessboard). Click it so it has keyboard focus.

| Key | Action |
|---|---|
| `s` | capture one sample: target visible + robot TF at the image time + diverse enough |
| `a` | toggle auto-sample: captures whenever the target is visible and the pose is diverse enough |
| `c` | solve, once `samples.required` samples are collected |
| `d` | delete all samples |
| `q` | quit |

Procedure:
1. Start the camera driver and the robot (TF `base_frame → tool_frame` must be live).
2. Move the arm to a pose where the target is fully visible, **hold still**, and press `s`. Hold still because each
   sample pairs the image with the robot TF at the image timestamp.
3. Repeat for 20 or more poses. Vary **rotation** on all three axes (tilt and roll the target, about 15–30°), not
   just position, and cover the image area and a range of distances. Planar or repeated poses give a weak solution.
   Poses too close to an earlier sample are rejected with `Pose rejected: not diverse enough`.
4. Press `c`. The node logs the result and its quality, writes `output.file_path` and publishes the result as a
   static TF. To calibrate again, restart the node.

### Reading the result

Example log (values are illustrative):

```
--- Calibration Evaluation ---
Calibration Quality : Very Good
Samples Used: 20
Reprojection Error: 0.7 pixels
Translation RMS Error: 4.1 mm
Rotation RMS Error: 0.35 degrees
```

- **Reprojection error**: how well the detected target matches its own pose estimate, so it shows detection
  quality, not the hand-eye result. Grades: < 0.5 px Excellent, < 1 Very Good, < 2 Good, < 5 Acceptable, else Poor.
- **Translation / rotation RMS**: the hand-eye check. With the solved transform, the target must come out at
  the same place in every sample (in the base frame for eye-in-hand, in the tool frame for eye-to-hand). These
  are the spreads around that place. A few mm and below about 1° is a good result. Larger values mean bad samples,
  a loose target, a wrong `marker_size` or robot TF that does not match the image time.

Output file (`hand_eye_calibration_output.yaml`):

```yaml
hand_eye_calibration:
  parent_frame: base_link      # tool_frame for eye_in_hand
  child_frame: camera_link
  transform:
    translation: {x: ..., y: ..., z: ...}
    rotation: {x: ..., y: ..., z: ..., w: ...}
```

The static TF lasts only while the node runs. To use the calibration afterwards, publish it yourself, for example:

```bash
ros2 run tf2_ros static_transform_publisher --x X --y Y --z Z --qx QX --qy QY --qz QZ --qw QW \
  --frame-id base_link --child-frame-id camera_link
```

or put it in your robot's URDF as a fixed joint.

## 5. Send the arm to an ArUco marker

`aruco_goal_publisher` detects a marker, finds where it is in the robot base frame through TF (so the calibration
TF from step 4 must be published), and publishes a goal pose for the arm.

```bash
ros2 launch universal_handeye aruco_goal_publisher.launch.py
# or: ... config_file:=/path/to/goal_publisher.yaml
```

```
image → detect ArUco → TF camera.frame → aruco_marker → lookup base_frame → aruco_marker
      → × tcp_offset (optional) → /goal_pose (PoseStamped in base_frame) + TF base_frame → aruco_goal
```

`config/goal_publisher.yaml`: camera topics and `camera.frame` (the same frame name the calibration used),
`robot.base_frame`, `target.*`, `goal.topic` (default `/goal_pose`), `goal.rate_hz`, and
`tcp_offset.{enabled, translation, rotation_rpy}`. The offset is applied in the marker frame. It is used to stop
the TCP short of the marker or to turn it to face the marker. Example: `translation: [0, 0, -0.10]` with
`rotation_rpy: [0, 3.14159, 0]` gives 10 cm in front of the marker face, turned to face it.

Your motion planner (for example MoveIt 2) or controller subscribes to `goal.topic`. To feed a controller that
reads another topic, run the republisher:

```bash
ros2 run universal_handeye goal_republisher_node --ros-args \
  -p input_pose_topic:=/goal_pose -p output_pose_topic:=/quest/right/pose
```

Check in RViz first: show TF and the `aruco_goal` frame, and confirm that it sits on the real marker before
letting the arm move.

## How it works

| Module | Job |
|---|---|
| `main.py` (`handeye_node`) | calibration node: timer loop, display, keyboard, capture, solve |
| `camera.py` | subscribes to image + `CameraInfo`, keeps the latest image, intrinsics and distortion |
| `detector.py` | finds the chessboard (`findChessboardCorners` + `solvePnP`) or ArUco marker (`estimatePoseSingleMarkers`) → `T_cam_target` |
| `tf_manager.py` | looks up `base_frame → tool_frame` at the image timestamp |
| `sampler.py` | stores robot/target pose pairs and rejects samples too close to earlier ones |
| `solver.py` | builds the inputs for `cv2.calibrateHandEye()`, inverting the robot pose for eye-to-hand |
| `evaluator.py` | reprojection error and transform-consistency RMS |
| `publisher.py` | writes the YAML and publishes the static TF |
| `goal_pub.py` (`goal_publisher_node`) | ArUco → `/goal_pose` through TF |
| `quest_republisher.py` (`goal_republisher_node`) | republishes a `PoseStamped` to another topic |
| `object_track.py` | older version of the goal publisher: loads `hand_eye_calibration_outputv1.yaml` from the current directory and publishes `/goal_pose` + TF `detected_target`. Not installed as an executable; `goal_pub.py` replaces it |
| `scripts/aruco_pub.py` | standalone demo: USB camera (OpenCV `VideoCapture`) → ArUco board TF + left/right grasp frames and `/aruco_pose`. Hard-coded frames and sizes |

Math: in the solver, eye-in-hand solves `T_tool_cam` from pairs (`T_base_tool`, `T_cam_target`). Eye-to-hand passes
`T_tool_base = T_base_tool⁻¹` instead, and the same OpenCV call then returns `T_base_cam`. Using it:
`T_base_marker = T_base_cam · T_cam_marker`.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Waiting for camera data...` | wrong `image_topic` / `camera_info_topic`, or no `CameraInfo` yet |
| `TF lookup failed: ... extrapolation into the future` | the robot's TF is older than the image. Hold the arm still and press `s` again, or check that robot and camera clocks agree |
| `Pose rejected: not diverse enough` | move further or rotate more, or lower `samples.min_translation` / `min_rotation` |
| `Cannot run calibration. Need N samples` | collect `samples.required` samples first |
| Marker not detected | wrong `target.dictionary` or `marker_id`, glare, marker too small in the image |
| Large RMS error | loose target, wrong `marker_size` / `square_size`, too little rotation between samples, robot moving during capture. Try another `solver.method` (`PARK`, `DANIILIDIS`) and compare |
| Goal in the wrong place | `camera.frame` differs between the calibration and the goal publisher, or the camera frame has two TF parents |

## License

MIT (see `package.xml`).
