import rclpy
import yaml
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import cv2 # Assuming you have a camera feed and detector logic

# --- Helper Functions ---

def load_calibration_from_yaml(filepath):
    """
    Loads the hand-eye calibration result from a YAML file and returns
    the 4x4 transformation matrix.
    """
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)

    calib_data = data['hand_eye_calibration']
    t = calib_data['transform']['translation']
    q = calib_data['transform']['rotation']

    translation = np.array([t['x'], t['y'], t['z']])
    # Scipy uses (x, y, z, w) for quaternions, which matches the YAML file.
    rotation = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']])

    T_matrix = np.eye(4)
    T_matrix[:3, :3] = rotation.as_matrix()
    T_matrix[:3, 3] = translation
    
    parent_frame = calib_data['parent_frame']

    return T_matrix, parent_frame

def rvec_tvec_to_matrix(rvec, tvec):
    """Converts rvec and tvec to a 4x4 homogeneous matrix."""
    mat = np.eye(4)
    R, _ = cv2.Rodrigues(rvec)
    mat[:3, :3] = R
    mat[:3, 3] = tvec.flatten()
    return mat

def matrix_to_posestamped(matrix, frame_id, stamp):
    """Converts a 4x4 transformation matrix to a PoseStamped message."""
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = stamp
    pose_stamped.header.frame_id = frame_id

    # Translation
    pose_stamped.pose.position.x = matrix[0, 3]
    pose_stamped.pose.position.y = matrix[1, 3]
    pose_stamped.pose.position.z = matrix[2, 3]

    # Rotation (from rotation matrix to quaternion)
    quat = Rotation.from_matrix(matrix[:3, :3]).as_quat() # returns x, y, z, w
    pose_stamped.pose.orientation.x = quat[0]
    pose_stamped.pose.orientation.y = quat[1]
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]

    return pose_stamped


class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker')
        
        # 1. LOAD THE CALIBRATION RESULT AT STARTUP
        # ==========================================
        calibration_file = 'hand_eye_calibration_outputv1.yaml'
        self.get_logger().info(f"Loading calibration from {calibration_file}...")
        self.T_base_cam, self.robot_base_frame = load_calibration_from_yaml(calibration_file)
        self.get_logger().info("Calibration loaded successfully.")

        # Initialize Camera Interface
        self.camera = CameraInterface(self)

        # Initialize Detector
        self.detector = CalibrationTargetDetector(self)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher for the goal pose
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Timer to run the detection loop more frequently
        self.timer = self.create_timer(0.1, self.detection_loop)

    def detection_loop(self):
        image = self.camera.get_image()
        image_stamp = self.camera.get_image_stamp()
        cam_matrix = self.camera.get_camera_matrix()
        dist_coeffs = self.camera.get_dist_coeffs()

        if image is None or image_stamp is None or cam_matrix is None or dist_coeffs is None:
            self.get_logger().info(
                "Waiting for camera data...",
                throttle_duration_sec=5
            )
            return

        # 2. DETECT THE TARGET FROM THE CAMERA
        # ===================================================
        success, rvec, tvec, _, _ = self.detector.detect(
            image,
            cam_matrix,
            dist_coeffs
        )

        if not success:
            self.get_logger().debug("No ArUco marker detected.", throttle_duration_sec=1)
            return

        T_cam_target = rvec_tvec_to_matrix(rvec, tvec)

        # 3. TRANSFORM THE TARGET POSE TO THE ROBOT'S BASE FRAME
        # ========================================================
        # This is the core calculation: T_base_target = T_base_cam @ T_cam_target
        T_base_target = self.T_base_cam @ T_cam_target
        
        # 4. PREPARE AND PUBLISH THE GOAL FOR THE ROBOT
        # ==============================================
        # Convert the final matrix into a PoseStamped message
        current_stamp = self.get_clock().now().to_msg()
        goal_pose_msg = matrix_to_posestamped(
            T_base_target, 
            self.robot_base_frame, 
            current_stamp
        )
        
        self.goal_pose_pub.publish(goal_pose_msg)
        self.get_logger().info(f"Published goal pose for target in '{self.robot_base_frame}' frame.")

        # 5. PUBLISH THE GOAL AS A TF TRANSFORM
        # =======================================
        t = TransformStamped()
        t.header.stamp = current_stamp
        t.header.frame_id = self.robot_base_frame
        t.child_frame_id = 'detected_target'

        t.transform.translation.x = T_base_target[0, 3]
        t.transform.translation.y = T_base_target[1, 3]
        t.transform.translation.z = T_base_target[2, 3]

        quat = Rotation.from_matrix(T_base_target[:3, :3]).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published TF transform for 'detected_target' relative to '{self.robot_base_frame}'.")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
