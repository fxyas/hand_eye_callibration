import cv2
import numpy as np
from scipy.spatial.transform import Rotation


class CalibrationSample:
    """Data class to hold a single calibration sample."""

    def __init__(self, robot_pose, target_pose, image_points=None, object_points=None):
        self.robot_pose = robot_pose      # 4x4 matrix (T_base_tool)
        self.target_pose = target_pose    # 4x4 matrix (T_cam_target)
        self.image_points = image_points
        self.object_points = object_points


class CalibrationSampler:
    """Collects and manages calibration samples."""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        self.samples = []

        self.node.declare_parameter("samples.required", 20)
        self.node.declare_parameter("samples.min_translation", 0.1)  # meters
        self.node.declare_parameter("samples.min_rotation", 15.0)  # degrees

        self.required_samples = self.node.get_parameter("samples.required").value
        self.min_translation = self.node.get_parameter("samples.min_translation").value
        self.min_rotation_rad = np.deg2rad(
            self.node.get_parameter("samples.min_rotation").value
        )

        self.logger.info(
            f"Sampler initialized. Required samples: {self.required_samples}"
        )

    def add_sample(
        self,
        robot_pose_stamped,
        target_rvec,
        target_tvec,
        image_points,
        object_points
    ):
        """Add a new sample if it is diverse enough."""
        if target_rvec is None or target_tvec is None:
            self.logger.warn(
                "Cannot add sample, target pose is invalid (rvec or tvec is None)."
            )
            return False

        robot_pose_mat = self._transform_stamped_to_matrix(robot_pose_stamped)
        target_pose_mat = self._rvec_tvec_to_matrix(target_rvec, target_tvec)

        is_diverse, reason = self._is_diverse(robot_pose_mat)
        if not is_diverse:
            self.logger.warn(
                f"Pose rejected: not diverse enough. Reason: {reason}",
                throttle_duration_sec=2
            )
            return False

        sample = CalibrationSample(
            robot_pose_mat,
            target_pose_mat,
            image_points,
            object_points
        )
        self.samples.append(sample)
        self.logger.info(f"Sample {len(self.samples)}/{self.required_samples} collected.")
        return True

    def _is_diverse(self, new_pose_mat):
        """Check if a new pose is sufficiently different from existing ones."""
        if not self.samples:
            return True, ""

        for sample in self.samples:
            existing_pose_mat = sample.robot_pose

            # Check translation distance
            translation_diff = np.linalg.norm(
                new_pose_mat[:3, 3] - existing_pose_mat[:3, 3]
            )
            if translation_diff < self.min_translation:
                return False, f"translation diff {translation_diff:.3f}m < {self.min_translation:.3f}m"

            # Check rotation difference
            R_new = new_pose_mat[:3, :3]
            R_existing = existing_pose_mat[:3, :3]
            R_diff = R_new @ R_existing.T

            angle_diff_rad = np.arccos(
                np.clip((np.trace(R_diff) - 1) / 2.0, -1.0, 1.0)
            )
            if angle_diff_rad < self.min_rotation_rad:
                return False, f"rotation diff {np.rad2deg(angle_diff_rad):.1f}deg < {np.rad2deg(self.min_rotation_rad):.1f}deg"

        return True, ""

    def get_samples(self):
        """Return the collected samples."""
        return self.samples

    def is_ready(self):
        """Check if enough samples have been collected."""
        return len(self.samples) >= self.required_samples

    def clear(self):
        """Clear all collected samples."""
        self.samples = []
        self.logger.info("All calibration samples cleared.")

    @staticmethod
    def _transform_stamped_to_matrix(transform):
        """Convert a geometry_msgs/TransformStamped to a 4x4 numpy matrix."""
        t = transform.transform.translation
        r = transform.transform.rotation
        rotation = Rotation.from_quat([r.x, r.y, r.z, r.w])

        mat = np.eye(4)
        mat[:3, :3] = rotation.as_matrix()
        mat[:3, 3] = np.array([t.x, t.y, t.z])
        return mat

    @staticmethod
    def _rvec_tvec_to_matrix(rvec, tvec):
        """
        Convert solvePnP output into a 4x4 homogeneous transform.

        The matrix maps points from the target/object frame into the camera
        frame, often written as T_cam_target.
        """
        mat = np.eye(4)
        R, _ = cv2.Rodrigues(rvec)
        mat[:3, :3] = R
        mat[:3, 3] = tvec.flatten()
        return mat
