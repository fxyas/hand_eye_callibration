import cv2
import numpy as np


class CalibrationTargetDetector:
    """Detect calibration target poses relative to the camera."""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        # Declare parameters for calibration target
        self.node.declare_parameter("target.type", "chessboard")
        self.node.declare_parameter("target.rows", 7)
        self.node.declare_parameter("target.columns", 9)
        self.node.declare_parameter("target.square_size", 0.025)  # meters
        self.node.declare_parameter("target.marker_size", 0.05)  # meters
        self.node.declare_parameter("target.dictionary", "DICT_4X4_50")
        self.node.declare_parameter("target.marker_id", -1)

        # Get parameter values
        self.target_type = self.node.get_parameter("target.type").value
        self.rows = self.node.get_parameter("target.rows").value
        self.columns = self.node.get_parameter("target.columns").value
        self.square_size = self.node.get_parameter("target.square_size").value
        self.marker_size = self.node.get_parameter("target.marker_size").value
        self.dictionary_name = self.node.get_parameter("target.dictionary").value
        self.marker_id = self.node.get_parameter("target.marker_id").value

        self.object_points = None
        self.aruco_dict = None
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self._initialize_target_specific_params()

        self.logger.info(f"Calibration target detector initialized for type: {self.target_type}")

    def _initialize_target_specific_params(self):
        """Initialize parameters specific to the chosen target type."""
        if self.target_type == "chessboard":
            self.object_points = np.zeros(
                (self.rows * self.columns, 3),
                np.float32
            )
            self.object_points[:, :2] = (
                np.mgrid[0:self.columns, 0:self.rows]
                .T.reshape(-1, 2)
                * self.square_size
            )

        elif self.target_type == "aruco":
            try:
                self.aruco_dict = cv2.aruco.getPredefinedDictionary(
                    getattr(cv2.aruco, self.dictionary_name)
                )
            except AttributeError:
                self.logger.error(
                    f"Invalid ArUco dictionary: {self.dictionary_name}"
                )
                self.aruco_dict = None

        else:
            self.logger.error(
                f"Unsupported target type: {self.target_type}"
            )

    def detect(self, image, camera_matrix, dist_coeffs):
        """Detect the calibration target and estimate its pose."""
        if image is None or camera_matrix is None or dist_coeffs is None:
            self.logger.warn(
                "Image, camera matrix, or distortion coefficients are missing."
            )
            return False, None, None, None, None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if self.target_type == "chessboard":
            return self._detect_chessboard(gray, camera_matrix, dist_coeffs)
        elif self.target_type == "aruco":
            return self._detect_aruco(
                gray,
                camera_matrix,
                dist_coeffs
            )
        else:
            self.logger.error(
                "Unsupported target type"
            )
            return False, None, None, None, None

    def _detect_chessboard(self, gray_image, camera_matrix, dist_coeffs):
        """Detect a chessboard pattern."""
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH |
            cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        ret, corners = cv2.findChessboardCorners(
            gray_image,
            (self.columns, self.rows),
            flags
        )

        if ret:
            # Refine corner locations
            corners = cv2.cornerSubPix(
                gray_image,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )

            # Solve for the pose of the chessboard
            ret, rvec, tvec = cv2.solvePnP(
                self.object_points,
                corners,
                camera_matrix,
                dist_coeffs
            )

            if ret:
                self.logger.debug("Chessboard detected and pose estimated.")
                return True, rvec, tvec, corners, self.object_points
            else:
                self.logger.warn("Failed to solvePnP for chessboard.")
                return False, None, None, None, None
        else:
            self.logger.debug("Chessboard not found.")
            return False, None, None, None, None

    def _detect_aruco(self, gray_image, camera_matrix, dist_coeffs):
        """Detect a single ArUco marker."""
        if self.aruco_dict is None:
            self.logger.error("ArUco dictionary not initialized.")
            return False, None, None, None, None

        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray_image)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray_image,
                self.aruco_dict,
                parameters=self.aruco_params
            )

        if ids is not None and len(ids) > 0:
            # Filter for a specific marker ID if configured
            if self.marker_id != -1:
                try:
                    idx = np.where(ids == self.marker_id)[0][0]
                    target_corners = np.array([corners[idx]])
                    target_ids = np.array([[self.marker_id]])
                except IndexError:
                    self.logger.debug(
                        f"Specific ArUco marker ID {self.marker_id} not found."
                    )
                    return False, None, None, None, None
            else:
                # Use the first detected marker if no specific ID is given
                target_corners = np.array([corners[0]])
                target_ids = np.array([[ids[0][0]]])

            # Estimate pose for the detected marker(s)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                target_corners,
                self.marker_size,
                camera_matrix,
                dist_coeffs
            )

            if rvecs is not None and len(rvecs) > 0:
                self.logger.debug(
                    f"ArUco marker ID {target_ids[0][0]} detected and pose estimated."
                )

                marker_object_points = np.array([
                    [-self.marker_size / 2, self.marker_size / 2, 0],
                    [self.marker_size / 2, self.marker_size / 2, 0],
                    [self.marker_size / 2, -self.marker_size / 2, 0],
                    [-self.marker_size / 2, -self.marker_size / 2, 0]
                ], dtype=np.float32)

                return (
                    True,
                    rvecs[0].reshape(3, 1),
                    tvecs[0].reshape(3, 1),
                    target_corners[0],
                    marker_object_points
                )
            else:
                self.logger.warn("Failed to estimate pose for ArUco marker.")
                return False, None, None, None, None
        else:
            self.logger.debug("No ArUco markers found.")
            return False, None, None, None, None
