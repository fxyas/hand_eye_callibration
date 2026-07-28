import yaml
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
import tf2_ros


class CalibrationPublisher:
    """Saves and publishes the final calibration result."""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        self.node.declare_parameter("output.file_path", "hand_eye_calibration.yaml")
        self.output_path = self.node.get_parameter("output.file_path").value

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)

    def publish(self, transform_matrix, parent_frame, child_frame):
        """Save the transform to YAML and publish it as static TF."""
        if transform_matrix is None:
            self.logger.error("Cannot publish, transform matrix is None.")
            return

        if transform_matrix.shape != (4, 4):
            self.logger.error("Invalid transform matrix. Expected 4x4.")
            return

        self.save_to_yaml(transform_matrix, parent_frame, child_frame)
        self.publish_static_tf(transform_matrix, parent_frame, child_frame)

    def save_to_yaml(self, transform_matrix, parent_frame, child_frame):
        """Save the transformation to a YAML file."""
        translation = transform_matrix[:3, 3]
        quat = Rotation.from_matrix(transform_matrix[:3, :3]).as_quat()  # x, y, z, w

        data = {
            'hand_eye_calibration': {
                'parent_frame': parent_frame,
                'child_frame': child_frame,
                'transform': {
                    'translation': {
                        'x': float(translation[0]),
                        'y': float(translation[1]),
                        'z': float(translation[2]),
                    },
                    'rotation': {
                        'x': float(quat[0]),
                        'y': float(quat[1]),
                        'z': float(quat[2]),
                        'w': float(quat[3]),
                    }
                }
            }
        }

        try:
            with open(self.output_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.logger.info(f"Calibration result saved to: {self.output_path}")
        except Exception as e:
            self.logger.error(f"Failed to save calibration to YAML file: {e}")

    def publish_static_tf(self, transform_matrix, parent_frame, child_frame):
        """Publish the transformation as a static TF transform."""
        t = TransformStamped()

        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        translation = transform_matrix[:3, 3]
        quat = Rotation.from_matrix(transform_matrix[:3, :3]).as_quat()  # x, y, z, w

        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.static_broadcaster.sendTransform(t)
        self.logger.info(
            f"Published static transform from '{parent_frame}' to '{child_frame}'."
        )
