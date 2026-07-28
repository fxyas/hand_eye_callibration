import rclpy
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformException


class TFManager:
    """Manage TF2 transformations for the robot."""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        # Declare parameters for robot frames
        self.node.declare_parameter("robot.base_frame", "base_link")
        self.node.declare_parameter("robot.tool_frame", "tool0")

        self.base_frame = self.node.get_parameter("robot.base_frame").value
        self.tool_frame = self.node.get_parameter("robot.tool_frame").value

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.logger.info(
            f"TF Manager initialized. Listening for transform from "
            f"'{self.base_frame}' to '{self.tool_frame}'."
        )

    def get_robot_pose(self, timestamp):
        """Get the robot base-to-tool transform at a specific time."""
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                timestamp,
                timeout=Duration(seconds=0.1)
            )
            self.logger.debug(
                f"Successfully retrieved robot pose for timestamp: {timestamp}"
            )
            return transform_stamped

        except TransformException as ex:
            self.logger.warn(
                f"TF lookup failed: {ex}",
                throttle_duration_sec=5
            )
            return None

    def get_base_frame(self):
        return self.base_frame

    def get_tool_frame(self):
        return self.tool_frame

    def is_ready(self):
        """Return True if the required robot transform is currently available."""
        return self.tf_buffer.can_transform(
            self.base_frame,
            self.tool_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.1)
        )
