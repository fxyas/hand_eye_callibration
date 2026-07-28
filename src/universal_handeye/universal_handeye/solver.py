import cv2
import numpy as np


class HandEyeSolver:
    """Solves the AX=XB hand-eye calibration problem."""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        self.node.declare_parameter(
            "solver.method",
            "TSAI"
        )

        self.method_str = (
            self.node.get_parameter("solver.method")
            .value
            .upper()
        )

        self.cv_method = self._opencv_method()

        self.logger.info(
            f"Hand-eye solver initialized with method: {self.method_str}"
        )

    def _opencv_method(self):
        methods = {
            "TSAI": cv2.CALIB_HAND_EYE_TSAI,
            "PARK": cv2.CALIB_HAND_EYE_PARK,
            "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
            "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
            "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS
        }

        return methods.get(
            self.method_str,
            cv2.CALIB_HAND_EYE_TSAI
        )

    def solve(self, samples, mode):
        """Perform hand-eye calibration."""
        if not samples or len(samples) < 5:
            self.logger.error(
                "Cannot solve, not enough samples provided (minimum 5)."
            )
            return None

        R_gripper2base, t_gripper2base = [], []
        R_target2cam, t_target2cam = [], []

        try:
            if mode == "eye_in_hand":
                self.logger.info("Preparing samples for Eye-in-Hand calibration.")
                for sample in samples:
                    T_base_tool = sample.robot_pose
                    T_cam_target = sample.target_pose

                    R_gripper2base.append(T_base_tool[:3, :3])
                    t_gripper2base.append(T_base_tool[:3, 3].reshape(3, 1))
                    R_target2cam.append(T_cam_target[:3, :3])
                    t_target2cam.append(T_cam_target[:3, 3].reshape(3, 1))

            elif mode == "eye_to_hand":
                self.logger.info("Preparing samples for Eye-to-Hand calibration.")
                for sample in samples:
                    T_tool_base = np.linalg.inv(sample.robot_pose)
                    T_cam_target = sample.target_pose

                    R_gripper2base.append(T_tool_base[:3, :3])
                    t_gripper2base.append(T_tool_base[:3, 3].reshape(3, 1))
                    R_target2cam.append(T_cam_target[:3, :3])
                    t_target2cam.append(T_cam_target[:3, 3].reshape(3, 1))
            else:
                self.logger.error(f"Invalid calibration mode: {mode}")
                return None

            self.logger.info(
                f"Solving hand-eye calibration with {len(samples)} samples."
            )

            R, t = cv2.calibrateHandEye(
                R_gripper2base=R_gripper2base,
                t_gripper2base=t_gripper2base,
                R_target2cam=R_target2cam,
                t_target2cam=t_target2cam,
                method=self.cv_method
            )

            result_mat = self._matrix(R, t)

            if mode == "eye_in_hand":
                self.logger.info("Solved transform: tool frame -> camera frame.")
            else:
                self.logger.info("Solved transform: base frame -> camera frame.")
            return result_mat

        except cv2.error as e:
            self.logger.error(f"OpenCV Error during hand-eye calibration: {e}")
            return None

    @staticmethod
    def _matrix(R, t):
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.flatten()
        return T
