import numpy as np
import cv2
from scipy.spatial.transform import Rotation


class CalibrationEvaluator:
    """Evaluates the quality of the hand-eye calibration."""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

    def evaluate(self, samples, hand_eye_transform, camera_matrix, dist_coeffs, mode):
        """Calculate and print calibration error metrics."""
        if not samples or hand_eye_transform is None:
            self.logger.error("Cannot evaluate. Invalid inputs provided.")
            return

        reprojection_error = self.calculate_reprojection_error(
            samples, hand_eye_transform, camera_matrix, dist_coeffs, mode
        )

        rms_error_trans, rms_error_rot = self.calculate_rms_error(
            samples, hand_eye_transform, mode
        )

        self.logger.info("--- Calibration Evaluation ---")

        grade = "Unknown"

        if reprojection_error is None:
            self.logger.warn(
                "Reprojection error could not be computed."
            )
        else:
            if reprojection_error < 0.5:
                grade = "Excellent"

            elif reprojection_error < 1.0:
                grade = "Very Good"

            elif reprojection_error < 2.0:
                grade = "Good"

            elif reprojection_error < 5.0:
                grade = "Acceptable"

            else:
                grade = "Poor"

            self.logger.info(
                f"Calibration Quality : {grade}"
            )
        self.logger.info(f"Samples Used: {len(samples)}")
        if reprojection_error is not None:
            self.logger.info(
                f"Reprojection Error: {reprojection_error:.4f} pixels"
            )
        if rms_error_trans is not None:
            self.logger.info(
                f"Translation RMS Error: {rms_error_trans * 1000:.4f} mm"
            )
        if rms_error_rot is not None:
            self.logger.info(
                f"Rotation RMS Error: {np.rad2deg(rms_error_rot):.4f} degrees"
            )
        self.logger.info("------------------------------")

        return {

            "reprojection_error": reprojection_error,
            "translation_rms": rms_error_trans,
            "rotation_rms": rms_error_rot,
            "quality": grade,
            "samples": len(samples)
        }

    def calculate_reprojection_error(
        self,
        samples,
        hand_eye_transform,
        camera_matrix,
        dist_coeffs,
        mode
    ):
        """Calculate the average target reprojection error."""
        total_error = 0
        total_points = 0

        for sample in samples:
            if sample.image_points is None or sample.object_points is None:
                continue  # Cannot calculate for this sample

            # We need to project object points from the target frame to the camera's image plane.
            # This requires the pose of the target relative to the camera (T_cam_target).
            T_cam_target = sample.target_pose
            rvec, _ = cv2.Rodrigues(T_cam_target[:3, :3])
            tvec = T_cam_target[:3, 3].reshape(3, 1)

            projected_points, _ = cv2.projectPoints(
                sample.object_points, rvec, tvec, camera_matrix, dist_coeffs
            )

            error = cv2.norm(
                sample.image_points.reshape(-1, 2),
                projected_points.reshape(-1, 2),
                cv2.NORM_L2
            )
            total_error += error
            total_points += len(sample.object_points)

        if total_points == 0:
            self.logger.warn(
                "Could not calculate reprojection error: no object/image "
                "points found in samples."
            )
            return None

        mean_error = total_error / total_points
        return mean_error

    def calculate_rms_error(self, samples, hand_eye_transform, mode):
        """Calculate transform-consistency RMS errors."""
        reference_poses = []

        if len(samples) < 2:
            return None, None

        for sample in samples:
            if mode == "eye_in_hand":
                # The calibration is good when the target pose in the base
                # frame stays fixed across all robot poses.
                reference_pose = (
                    sample.robot_pose
                    @ hand_eye_transform
                    @ sample.target_pose
                )
            elif mode == "eye_to_hand":
                # The calibration is good when the target pose in the tool
                # frame stays fixed across all robot poses.
                reference_pose = (
                    np.linalg.inv(sample.robot_pose)
                    @ hand_eye_transform
                    @ sample.target_pose
                )
            else:
                self.logger.error(f"Invalid calibration mode: {mode}")
                return None, None

            reference_poses.append(reference_pose)

        if len(reference_poses) < 2:
            return None, None

        translations = np.array([pose[:3, 3] for pose in reference_poses])
        mean_translation = np.mean(translations, axis=0)
        errors_trans = [
            np.linalg.norm(translation - mean_translation)
            for translation in translations
        ]

        mean_rotation = self._mean_rotation([pose[:3, :3] for pose in reference_poses])
        errors_rot = [
            self._rotation_angle(mean_rotation.T @ pose[:3, :3])
            for pose in reference_poses
        ]

        rms_trans = np.sqrt(np.mean(np.square(errors_trans)))
        rms_rot = np.sqrt(np.mean(np.square(errors_rot)))
        return rms_trans, rms_rot

    @staticmethod
    def _rotation_angle(rotation_matrix):
        return np.arccos(
            np.clip(
                (np.trace(rotation_matrix) - 1) / 2,
                -1,
                1
            )
        )

    @staticmethod
    def _mean_rotation(rotation_matrices):
        quaternions = []

        for rotation_matrix in rotation_matrices:
            quaternion = Rotation.from_matrix(rotation_matrix).as_quat()
            if quaternions and np.dot(quaternion, quaternions[0]) < 0:
                quaternion = -quaternion
            quaternions.append(quaternion)

        mean_quaternion = np.mean(quaternions, axis=0)
        mean_quaternion /= np.linalg.norm(mean_quaternion)
        return Rotation.from_quat(mean_quaternion).as_matrix()
