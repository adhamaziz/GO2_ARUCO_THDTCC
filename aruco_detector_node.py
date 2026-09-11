#!/usr/bin/env python3
"""
Subscribes to a RealSense image + camera_info topic, detects ArUco markers,
estimates 6-DOF pose for each via solvePnP, and publishes:
  - annotated image (sensor_msgs/Image)          -> for viewing
  - marker poses (geometry_msgs/PoseArray)        -> pose in camera frame, per marker
  - marker ids (std_msgs/Int32MultiArray)         -> parallel-indexed to the pose array

Poses require camera_info (intrinsics) to arrive at least once -- the
RealSense driver publishes this automatically, no extra setup needed.
"""
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('image_topic', '/go2/aruco/image_annotated')
        self.declare_parameter('poses_topic', '/go2/aruco/poses')
        self.declare_parameter('ids_topic', '/go2/aruco/marker_ids')
        # Common dictionaries: DICT_4X4_50, DICT_5X5_100, DICT_6X6_250, DICT_ARUCO_ORIGINAL
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        # Physical side length of your printed markers, in meters. Get this
        # wrong and every pose will be wrong by the same scale factor.
        self.declare_parameter('marker_size', 0.05)

        input_topic = self.get_parameter('input_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        image_topic = self.get_parameter('image_topic').value
        poses_topic = self.get_parameter('poses_topic').value
        ids_topic = self.get_parameter('ids_topic').value
        dict_name = self.get_parameter('dictionary').value
        self.marker_size = self.get_parameter('marker_size').value

        self.get_logger().info(f'ArUco dictionary: {dict_name}, marker_size: {self.marker_size} m')

        aruco_dict_id = getattr(cv2.aruco, dict_name)
        # cv2.aruco's API changed across OpenCV versions (ArucoDetector class
        # introduced ~4.7). Support both so this doesn't silently break on a
        # future opencv-contrib-python upgrade.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._new_api = True
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self._new_api = False
            self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_id)
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        sub_qos = QoSProfile(depth=5)
        sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sub_qos.history = HistoryPolicy.KEEP_LAST

        pub_qos = QoSProfile(depth=5)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.history = HistoryPolicy.KEEP_LAST

        self.image_pub = self.create_publisher(Image, image_topic, pub_qos)
        self.poses_pub = self.create_publisher(PoseArray, poses_topic, pub_qos)
        self.ids_pub = self.create_publisher(Int32MultiArray, ids_topic, pub_qos)

        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, sub_qos)
        self.create_subscription(Image, input_topic, self.image_callback, sub_qos)

        self.get_logger().info(f'Subscribing to: {input_topic} and {camera_info_topic}')

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self._new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)

        pose_array = PoseArray()
        pose_array.header = msg.header
        id_msg = Int32MultiArray()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            id_msg.data = [int(i) for i in ids.flatten()]

            if self.camera_matrix is not None:
                half = self.marker_size / 2.0
                obj_points = np.array([
                    [-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0],
                ], dtype=np.float64)

                for corner in corners:
                    img_points = corner.reshape(-1, 2).astype(np.float64)
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_points, img_points, self.camera_matrix, self.dist_coeffs)
                    if not ok:
                        continue
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                       rvec, tvec, self.marker_size * 0.5)

                    pose = Pose()
                    pose.position.x = float(tvec[0])
                    pose.position.y = float(tvec[1])
                    pose.position.z = float(tvec[2])

                    rot_mat, _ = cv2.Rodrigues(rvec)
                    qw, qx, qy, qz = self._rotmat_to_quat(rot_mat)
                    pose.orientation.w = qw
                    pose.orientation.x = qx
                    pose.orientation.y = qy
                    pose.orientation.z = qz
                    pose_array.poses.append(pose)
            else:
                self.get_logger().warn(
                    'No camera_info received yet -- publishing detections without poses',
                    throttle_duration_sec=5.0)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header = msg.header
        self.image_pub.publish(img_msg)
        self.poses_pub.publish(pose_array)
        self.ids_pub.publish(id_msg)

    @staticmethod
    def _rotmat_to_quat(R):
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return qw, qx, qy, qz


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()