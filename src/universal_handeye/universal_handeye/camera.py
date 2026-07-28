import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from rclpy.qos import qos_profile_system_default


class CameraInterface:

    def __init__(self, node):
        self.node = node

        self.bridge = CvBridge()

        self.image = None
        self.image_stamp = None
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.has_received_image = False
        self.has_received_info = False

        # Parameters
        self.node.declare_parameter(
            "camera.image_topic",
            "/image_raw"
        )

        self.node.declare_parameter(
            "camera.camera_info_topic",
            "/camera_info"
        )

        self.node.declare_parameter(
            "display.show_image",
            True
        )

        self.node.declare_parameter(
            "camera.frame",
            "camera_link"
        )

        image_topic = self.node.get_parameter(
            "camera.image_topic"
        ).value

        info_topic = self.node.get_parameter(
            "camera.camera_info_topic"
        ).value

        self.show_image = self.node.get_parameter(
            "display.show_image"
        ).value

        self.camera_frame = self.node.get_parameter(
            "camera.frame"
        ).value

        self.image_sub = self.node.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_system_default
        )

        self.info_sub = self.node.create_subscription(
            CameraInfo,
            info_topic,
            self.info_callback,
            qos_profile_system_default
        )

        self.node.get_logger().info(
            "Camera interface started"
        )

    def image_callback(self, msg):
        if not self.has_received_image:
            self.node.get_logger().info("First image received.")
            self.has_received_image = True
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_stamp = msg.header.stamp
        except Exception as e:
            self.node.get_logger().error(f"Image conversion error: {e}")

    def info_callback(self, msg):
        self.camera_info = msg

        # Extract camera matrix (K) and distortion coefficients (D)
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

        if not self.has_received_info:
            self.node.get_logger().info(f"Camera Info received: {msg.width}x{msg.height}")
            self.has_received_info = True

    def get_image(self):
        """Return the latest camera image."""
        return self.image

    def get_image_stamp(self):
        """Return the timestamp of the latest camera image."""
        return self.image_stamp

    def get_camera_matrix(self):
        """Return the camera intrinsic matrix (K)."""
        return self.camera_matrix

    def get_dist_coeffs(self):
        """Return the camera distortion coefficients (D)."""
        return self.dist_coeffs

    def get_camera_info(self):
        """Return the raw CameraInfo message."""
        return self.camera_info

    def get_camera_frame(self):
        """Return the camera's TF frame name."""
        return self.camera_frame
