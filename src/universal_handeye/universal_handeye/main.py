import rclpy
import cv2

from rclpy.node import Node

from universal_handeye.camera import CameraInterface
from universal_handeye.detector import CalibrationTargetDetector
from universal_handeye.tf_manager import TFManager
from universal_handeye.sampler import CalibrationSampler
from universal_handeye.solver import HandEyeSolver
from universal_handeye.evaluator import CalibrationEvaluator
from universal_handeye.publisher import CalibrationPublisher


class UniversalHandEye(Node):

    def __init__(self):

        super().__init__(
            "universal_handeye"
        )
        self.declare_parameter(
            "mode",
            "eye_in_hand"
        )
        self.camera = CameraInterface(
            self
        )

        self.detector = CalibrationTargetDetector(
            self
        )

        self.tf_manager = TFManager(
            self
        )

        self.sampler = CalibrationSampler(
            self
        )

        self.solver = HandEyeSolver(
            self
        )

        self.evaluator = CalibrationEvaluator(
            self
        )

        self.publisher = CalibrationPublisher(
            self
        )

        self.calibration_done = False
        self.auto_sample_mode = False

        self.timer = self.create_timer(0.1, self.step)

    def step(self):
        """Run one timer-driven calibration step."""
        self.handle_image_and_input()

    def handle_image_and_input(self):
        """Handle target detection, display, and keyboard input."""
        image = self.camera.get_image()
        image_stamp = self.camera.get_image_stamp()
        cam_matrix = self.camera.get_camera_matrix()
        dist_coeffs = self.camera.get_dist_coeffs()
        cam_info = self.camera.get_camera_info()

        if image is None or image_stamp is None or cam_matrix is None or cam_info is None:
            self.get_logger().info(
                "Waiting for camera data...",
                throttle_duration_sec=5
            )
            return

        # Make a copy for drawing
        display_image = image.copy()

        success, rvec, tvec, image_points, object_points = self.detector.detect(
            image,
            cam_matrix,
            dist_coeffs
        )

        if success:
            # Draw detected pattern on the image
            if self.detector.target_type == 'chessboard' and image_points is not None:
                cv2.drawChessboardCorners(
                    display_image,
                    (self.detector.columns, self.detector.rows),
                    image_points,
                    success
                )
            else:  # ArUco
                cv2.drawFrameAxes(display_image, cam_matrix, dist_coeffs, rvec, tvec, 0.1)

        if self.auto_sample_mode and success:
            # Check if TF is ready before attempting to capture a sample in auto mode
            if self.tf_manager.is_ready():
                self.try_capture_sample(
                    success,
                    rvec,
                    tvec,
                    image_points,
                    object_points,
                    image_stamp
                )

        # UI and State Handling
        self.draw_ui(display_image)
        if self.camera.show_image:
            cv2.imshow("HandEye Camera", display_image)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.get_logger().info("Quit requested. Shutting down node.")
                raise KeyboardInterrupt # This will be caught by the main loop
            elif key == ord('s'):
                self.try_capture_sample(
                    success,
                    rvec,
                    tvec,
                    image_points,
                    object_points,
                    image_stamp
                )
            elif key == ord('c'):
                self.run_calibration()
            elif key == ord('d'):
                self.sampler.clear()

            elif key == ord('a'):
                self.auto_sample_mode = not self.auto_sample_mode
                status = "enabled" if self.auto_sample_mode else "disabled"
                self.get_logger().info(f"Auto-sample mode {status}.")

    def try_capture_sample(self, target_found, rvec, tvec, image_points, object_points, timestamp):
        if not target_found:
            self.get_logger().warn("Cannot capture sample: target not visible.")
            return

        if not self.tf_manager.is_ready():
            self.get_logger().warn(
                "Robot TF not available yet."
            )
            return

        robot_pose = self.tf_manager.get_robot_pose(timestamp)

        if robot_pose:
            success = self.sampler.add_sample(
                robot_pose,
                rvec,
                tvec,
                image_points,
                object_points
            )

            if success:
                self.get_logger().info(
                    "Sample captured successfully."
                )

    def run_calibration(self):
        if self.calibration_done:
            self.get_logger().warn(
                "Calibration already completed. Restart the node to perform a new calibration."
            )
            return

        if not self.sampler.is_ready():
            self.get_logger().warn(
                f"Cannot run calibration. Need {self.sampler.required_samples} "
                f"samples, but have {len(self.sampler.samples)}."
            )
            return

        self.get_logger().info("--- Starting Hand-Eye Calibration ---")
        samples = self.sampler.get_samples()

        mode = self.get_parameter("mode").value
        hand_eye_transform = self.solver.solve(samples, mode)

        if hand_eye_transform is not None:
            self.evaluator.evaluate(
                samples,
                hand_eye_transform,
                self.camera.get_camera_matrix(),
                self.camera.get_dist_coeffs(),
                mode
            )

            if mode == 'eye_in_hand':
                parent_frame = self.tf_manager.get_tool_frame()
                child_frame = self.camera.get_camera_frame()
            else:  # eye_to_hand
                parent_frame = self.tf_manager.get_base_frame()
                child_frame = self.camera.get_camera_frame()

            self.publisher.publish(hand_eye_transform, parent_frame, child_frame)

            self.calibration_done = True

    def draw_ui(self, image):
        auto_sample_status = "ON" if self.auto_sample_mode else "OFF"
        text = (
            f"Samples: {len(self.sampler.samples)}/"
            f"{self.sampler.required_samples} | "
            f"(A)uto-sample: {auto_sample_status} | "
            "(S)ave | (C)alibrate | (D)elete | "
            "(Q)uit"
        )
        cv2.putText(
            image,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )


def main(args=None):

    rclpy.init(args=args)

    node = UniversalHandEye()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if node.camera.show_image:  # Only destroy windows if they were created
        cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
