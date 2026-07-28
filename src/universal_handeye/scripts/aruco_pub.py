#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation


class ArucoTFPublisher(Node):

    def __init__(self):

        super().__init__("aruco_tf_publisher")

        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/aruco_pose",
            10
        )

        # -----------------------------
        # Camera
        # -----------------------------
        self.cap = cv2.VideoCapture(0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # -----------------------------
        # Marker size (meters)
        # -----------------------------
        self.marker_size = 0.05

        # -----------------------------
        # Dictionary
        # -----------------------------
        self.dictionary = aruco.getPredefinedDictionary(
            aruco.DICT_5X5_50
        )

        self.detector = aruco.ArucoDetector(
            self.dictionary,
            aruco.DetectorParameters()
        )

        # ==========================================================
        # CHANGE THESE TO YOUR CAMERA CALIBRATION
        # ==========================================================
        self.camera_matrix = np.array([
            [900.0,   0.0, 640.0],
            [0.0,   900.0, 360.0],
            [0.0,     0.0,   1.0]
        ])

        self.dist_coeffs = np.zeros((5, 1))
        # ==========================================================

        self.timer = self.create_timer(
            0.03,
            self.process_frame
        )

    def process_frame(self):

        ret, frame = self.cap.read()

        if not ret:
            return

        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is not None:

            aruco.drawDetectedMarkers(
                frame,
                corners,
                ids
            )

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )

            for i in range(len(ids)):

                marker_id = int(ids[i][0])

                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Draw XYZ axes
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    0.03
                )

                rot_matrix, _ = cv2.Rodrigues(rvec)

                quat = Rotation.from_matrix(rot_matrix).as_quat()

                tf_msg = TransformStamped()

                tf_msg.header.stamp = self.get_clock().now().to_msg()

                tf_msg.header.frame_id = "camera_color_optical_frame"

                tf_msg.child_frame_id = "aruco_board"

                tf_msg.transform.translation.x = float(tvec[0])
                tf_msg.transform.translation.y = float(tvec[1])
                tf_msg.transform.translation.z = float(tvec[2])

                tf_msg.transform.rotation.x = float(quat[0])
                tf_msg.transform.rotation.y = float(quat[1])
                tf_msg.transform.rotation.z = float(quat[2])
                tf_msg.transform.rotation.w = float(quat[3])

                self.tf_broadcaster.sendTransform(tf_msg)

                # Left target
                left_tf = TransformStamped()

                left_tf.header.stamp = self.get_clock().now().to_msg()
                left_tf.header.frame_id = "aruco_board"
                left_tf.child_frame_id = "left_grasp"

                left_tf.transform.translation.x = 0.0
                left_tf.transform.translation.y = 0.15
                left_tf.transform.translation.z = 0.0

                left_tf.transform.rotation.x = 0.0
                left_tf.transform.rotation.y = 0.0
                left_tf.transform.rotation.z = 0.0
                left_tf.transform.rotation.w = 1.0

                self.tf_broadcaster.sendTransform(left_tf)

                # Right target
                right_tf = TransformStamped()

                right_tf.header.stamp = self.get_clock().now().to_msg()
                right_tf.header.frame_id = "aruco_board"
                right_tf.child_frame_id = "right_grasp"

                right_tf.transform.translation.x = 0.0
                right_tf.transform.translation.y = -0.15
                right_tf.transform.translation.z = 0.0

                right_tf.transform.rotation.x = 0.0
                right_tf.transform.rotation.y = 0.0
                right_tf.transform.rotation.z = 0.0
                right_tf.transform.rotation.w = 1.0

                self.tf_broadcaster.sendTransform(right_tf)

                pose = PoseStamped()

                pose.header = tf_msg.header

                pose.pose.position.x = float(tvec[0])
                pose.pose.position.y = float(tvec[1])
                pose.pose.position.z = float(tvec[2])

                pose.pose.orientation.x = float(quat[0])
                pose.pose.orientation.y = float(quat[1])
                pose.pose.orientation.z = float(quat[2])
                pose.pose.orientation.w = float(quat[3])

                self.pose_pub.publish(pose)

                x = int(corners[i][0][:, 0].mean())
                y = int(corners[i][0][:, 1].mean())

                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

                cv2.putText(
                    frame,
                    f"ID:{marker_id}",
                    (x + 10, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"X:{tvec[0]:.2f}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"Y:{tvec[1]:.2f}",
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"Z:{tvec[2]:.2f}",
                    (20, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        cv2.imshow("ArUco Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main():

    rclpy.init()

    node = ArucoTFPublisher()

    rclpy.spin(node)


if __name__ == "__main__":
    main()