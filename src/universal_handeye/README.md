# universal_handeye

ROS 2 hand-eye calibration (eye-in-hand and eye-to-hand, chessboard or ArUco, OpenCV solvers) and an
ArUco → `/goal_pose` publisher for sending an arm to a marker.

Full documentation (install, configuration, calibration procedure, result quality, goal publisher,
troubleshooting): see the [repository README](../../README.md).

```bash
ros2 launch universal_handeye handeye_calibration.launch.py      # calibrate: s / a / c / d / q in the camera window
ros2 launch universal_handeye aruco_goal_publisher.launch.py     # ArUco marker → /goal_pose in the robot base frame
```
