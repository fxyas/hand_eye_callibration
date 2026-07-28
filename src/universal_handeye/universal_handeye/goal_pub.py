import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
from tf2_ros import TransformException
import tf_transformations

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from universal_handeye.camera import CameraInterface
from universal_handeye.detector import CalibrationTargetDetector


class ArucoGoalPublisher(Node):
    """Detect an ArUco marker and publish its pose in the robot base frame
    as the goal pose for the arm TCP to move to.

    Pipeline:
        Image
          -> Detect ArUco
          -> Publish TF: camera_link -> aruco_marker
          -> lookup_transform(base_link, aruco_marker)
          -> (optional) apply TCP offset/orientation
          -> Publish /goal_pose
    """

    def __init__(self):
        super().__init__("aruco_goal_publisher")

        self.camera = CameraInterface(self)
        self.detector = CalibrationTargetDetector(self)

        self.declare_parameter("robot.base_frame", "base_link")
        self.base_frame = self.get_parameter("robot.base_frame").value

        self.declare_parameter("goal.topic", "/goal_pose")
        self.declare_parameter("goal.rate_hz", 10.0)
        self.declare_parameter("goal.marker_frame", "aruco_marker")
        self.declare_parameter("goal.frame", "aruco_goal")
        goal_topic = self.get_parameter("goal.topic").value
        rate_hz = self.get_parameter("goal.rate_hz").value
        self.marker_frame = self.get_parameter("goal.marker_frame").value
        self.goal_frame = self.get_parameter("goal.frame").value

        # Optional offset applied after the base->marker lookup, e.g. to aim
        # the arm's TCP at a fixed pose relative to the marker rather than at
        # the marker face itself.
        self.declare_parameter("tcp_offset.enabled", False)
        self.declare_parameter("tcp_offset.translation", [0.0, 0.0, 0.0])
        self.declare_parameter("tcp_offset.rotation_rpy", [0.0, 0.0, 0.0])
        self.tcp_offset_enabled = self.get_parameter("tcp_offset.enabled").value
        tcp_translation = self.get_parameter("tcp_offset.translation").value
        tcp_rotation_rpy = self.get_parameter("tcp_offset.rotation_rpy").value
        self.tcp_offset = self._build_offset_matrix(tcp_translation, tcp_rotation_rpy)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)

        self.timer = self.create_timer(1.0 / rate_hz, self.step)

        self.get_logger().info(
            f"ArUco goal publisher started. Publishing '{goal_topic}' in frame "
            f"'{self.base_frame}'."
        )

    @staticmethod
    def _build_offset_matrix(translation, rotation_rpy):
        matrix = tf_transformations.euler_matrix(*rotation_rpy)
        matrix[:3, 3] = translation
        return matrix

    def step(self):
        image = self.camera.get_image()
        image_stamp = self.camera.get_image_stamp()
        cam_matrix = self.camera.get_camera_matrix()
        dist_coeffs = self.camera.get_dist_coeffs()

        if image is None or cam_matrix is None:
            self.get_logger().info(
                "Waiting for camera data...", throttle_duration_sec=5
            )
            return

        display_image = image.copy() if self.camera.show_image else None

        # --- Detect ArUco ---
        success, rvec, tvec, _, _ = self.detector.detect(
            image, cam_matrix, dist_coeffs
        )

        if not success:
            if display_image is not None:
                self._show(display_image)
            return

        if display_image is not None:
            cv2.drawFrameAxes(display_image, cam_matrix, dist_coeffs, rvec, tvec, 0.1)
            self._show(display_image)

        # --- Publish TF: camera_link -> aruco_marker ---
        self._publish_marker_tf(rvec, tvec, image_stamp)

        # --- lookup_transform(base_link, aruco_marker) ---
        base_to_marker = self._lookup_marker_in_base()
        if base_to_marker is None:
            return

        # --- (Optional) apply TCP offset/orientation ---
        if self.tcp_offset_enabled:
            base_to_marker = base_to_marker @ self.tcp_offset

        # --- Publish /goal_pose ---
        goal_pose = self._matrix_to_pose_stamped(base_to_marker, image_stamp)
        self.goal_pub.publish(goal_pose)
        self._broadcast_goal_tf(goal_pose)

    def _publish_marker_tf(self, rvec, tvec, stamp):
        """Broadcast the detected marker pose relative to the camera as a TF
        transform: camera_link -> aruco_marker."""
        camera_frame = self.camera.get_camera_frame()

        rot_matrix, _ = cv2.Rodrigues(rvec)
        marker_in_camera = np.eye(4)
        marker_in_camera[:3, :3] = rot_matrix
        marker_in_camera[:3, 3] = tvec.reshape(3)
        quat = tf_transformations.quaternion_from_matrix(marker_in_camera)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = camera_frame
        tf_msg.child_frame_id = self.marker_frame
        tf_msg.transform.translation.x = float(tvec[0])
        tf_msg.transform.translation.y = float(tvec[1])
        tf_msg.transform.translation.z = float(tvec[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])

        # Broadcast on /tf for other nodes/RViz, and also insert directly
        # into the local buffer so the lookup below does not have to wait on
        # a /tf pub-sub round trip.
        self.tf_broadcaster.sendTransform(tf_msg)
        self.tf_buffer.set_transform(tf_msg, "aruco_goal_publisher")

    def _lookup_marker_in_base(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.marker_frame,
                Time(),
                timeout=Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"TF lookup '{self.base_frame}' <- '{self.marker_frame}' failed: {ex}",
                throttle_duration_sec=5
            )
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        matrix = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix

    def _matrix_to_pose_stamped(self, matrix, stamp):
        quat = tf_transformations.quaternion_from_matrix(matrix)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(matrix[0, 3])
        pose.pose.position.y = float(matrix[1, 3])
        pose.pose.position.z = float(matrix[2, 3])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        return pose

    def _broadcast_goal_tf(self, goal_pose):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = goal_pose.header.stamp
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = self.goal_frame
        tf_msg.transform.translation.x = goal_pose.pose.position.x
        tf_msg.transform.translation.y = goal_pose.pose.position.y
        tf_msg.transform.translation.z = goal_pose.pose.position.z
        tf_msg.transform.rotation = goal_pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def _show(self, image):
        cv2.imshow("Aruco Goal Publisher", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = ArucoGoalPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if node.camera.show_image:
        cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
