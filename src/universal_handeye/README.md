# Universal Hand-Eye Calibration

ROS 2 package for estimating the fixed transform between a robot and a camera
from robot TF poses and camera observations of a calibration target.

Supported setups:

- Eye-in-hand: camera mounted on the end-effector. Output is `tool_frame -> camera_frame`.
- Eye-to-hand: camera fixed outside the robot, target mounted on the end-effector. Output is `base_frame -> camera_frame`.

Supported targets:

- Chessboard
- Single ArUco marker

Supported OpenCV solvers:

- Tsai
- Park
- Horaud
- Andreff
- Daniilidis

## Script Responsibilities

The package is split into small Python scripts. Each script has one main job:

- `camera.py`: subscribes to the camera image and `CameraInfo` topics, converts ROS images to OpenCV images, and stores camera intrinsics.
- `detector.py`: finds the calibration target in the camera image. It supports chessboard and ArUco targets and estimates `target -> camera` pose with OpenCV.
- `sampler.py`: stores calibration samples. Each sample contains one robot pose, one detected target pose, and the image/object points used for checking quality.
- `solver.py`: runs OpenCV `calibrateHandEye()` using the collected samples. It supports Tsai, Park, Horaud, Andreff, and Daniilidis methods.
- `evaluator.py`: checks calibration quality using reprojection error and RMS pose consistency error.
- `publisher.py`: saves the final transform to a YAML file and publishes it as a static TF.
- `tf_manager.py`: reads the robot transform from TF, usually `base_frame -> tool_frame`.
- `main.py`: coordinates the full workflow. It gets camera data, detects the target, captures samples, runs calibration, evaluates the result, and publishes the final transform.
- `__init__.py`: marks `universal_handeye` as a Python package. It does not contain calibration logic.

## Build

From the workspace root:

```bash
colcon build --packages-select universal_handeye
source install/setup.bash
```

## Configure

Edit `config/handeye.yaml` or pass another YAML file through the launch argument.

Important parameters:

- `mode`: `eye_in_hand` or `eye_to_hand`
- `camera.image_topic`: camera image topic
- `camera.camera_info_topic`: calibrated camera info topic
- `camera.frame`: camera TF frame used for the published result
- `robot.base_frame`: robot base frame
- `robot.tool_frame`: robot tool/end-effector frame
- `target.type`: `chessboard` or `aruco`
- `target.rows`, `target.columns`: chessboard inner-corner count
- `target.square_size`: chessboard square size in meters
- `target.marker_size`: ArUco marker side length in meters
- `solver.method`: `TSAI`, `PARK`, `HORAUD`, `ANDREFF`, or `DANIILIDIS`
- `samples.required`: minimum samples before solving
- `output.file_path`: YAML output path

## Run

```bash
ros2 launch universal_handeye handeye_calibration.launch.py
```

With a custom config:

```bash
ros2 launch universal_handeye handeye_calibration.launch.py config_file:=/path/to/handeye.yaml
```

## Operator Controls

When `display.show_image` is true, the camera window accepts:

- `s`: capture a sample if the target is visible and robot TF is available
- `c`: run calibration after enough samples are collected
- `d`: delete all collected samples
- `q`: quit

Move the robot through diverse poses before capturing samples. Good samples should vary both position and orientation; mostly planar or repeated poses usually produce weak calibrations.

## Outputs

After solving, the package:

- logs reprojection error and RMS pose consistency errors
- saves the calibrated transform to `output.file_path`
- publishes the result as a static TF

The saved YAML contains:

```yaml
hand_eye_calibration:
  parent_frame: tool0
  child_frame: camera_link
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

For eye-to-hand mode the parent frame is the configured robot base frame.

===========================
T_base_cam: The transform from base_link to camera_link. This is the result of your hand-eye calibration. It's static and never changes. You save it in a file and load it when your application starts.

T_cam_aruco: The transform from camera_link to the aruco_marker. Your camera, using OpenCV, calculates this in real-time. Every time the camera sees the marker in a new position, it computes a new rvec and tvec and gives you this transform. The detector.py script does exactly this.

T_base_tcp: The transform from base_link to right_tcp. Your robot's controller always knows this transform from its internal joint states (forward kinematics).

The goal is to move the right_tcp to the aruco_marker's location. To do this, the robot's motion planner needs to know: "Where is the ArUco marker relative to my base_link?"

We can calculate this target pose, let's call it T_base_aruco, by multiplying the transforms we know:

T_base_aruco = T_base_cam @ T_cam_aruco

This simple matrix multiplication is the key that connects what the camera sees to the robot's world.

The Step-by-Step Workflow
Here is the logical flow of an application that would perform this task:

Initialization:

Your application starts.
It loads the hand_eye_calibration_output.yaml file to get the T_base_cam matrix.
It starts a perception node that uses the camera to look for ArUco markers.
Detection:

The perception node sees an ArUco marker.
It uses cv2.aruco.estimatePoseSingleMarkers to calculate the marker's pose relative to the camera (T_cam_aruco).
Transformation:

Your application takes the two matrices and multiplies them:
python
 Show full code block 
# T_base_cam is the 4x4 matrix from your hand_eye_calibration_output.yaml
T_base_cam = load_calibration_from_yaml("hand_eye_calibration_output.yaml")

# T_cam_aruco is the 4x4 matrix from your camera's ArUco detection node
# This is calculated in real-time from rvec and tvec.
T_cam_aruco = get_aruco_pose_from_camera()

# Calculate the final goal pose for the robot's TCP
# This pose is relative to the robot's base_link.
T_base_aruco_goal = T_base_cam @ T_cam_aruco
Command:

The resulting T_base_aruco_goal matrix represents the desired pose for your right_tcp in the base_link frame.
You convert this matrix into a PoseStamped message.
You send this PoseStamped message as a goal to your robot's motion planner (e.g., MoveIt2).
Execution:

The motion planner calculates a trajectory for the arm.
The robot executes the motion, moving its right_tcp frame until it is perfectly aligned with the aruco_marker's pose in the real world.
In summary, the hand-eye calibration provides the missing link that allows you to translate coordinates from the "camera's world" into the "robot's world."