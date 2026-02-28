import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


class ArucoMarkerNode(Node):
    def __init__(self):
        super().__init__('aruco_marker_node')

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.got_camera_info = False

        # ArUco setup
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.marker_size = 0.10 # 10 cm macker sizw 

        self.get_logger().info("Aruco Marker Node started")


    def camera_info_callback(self, msg: CameraInfo):
        if self.got_camera_info:
            return

        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.got_camera_info = True

        self.get_logger().info("Camera calibration received")

    def image_callback(self, msg: Image):
        if not self.got_camera_info:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )

            for marker_id, rvec, tvec in zip(ids, rvecs, tvecs):
                cv2.aruco.drawAxis(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    0.03
                )

                pos_text = f"ID {marker_id[0]} | x:{tvec[0][0]:.2f} y:{tvec[0][1]:.2f} z:{tvec[0][2]:.2f}"
                cv2.putText(
                    frame,
                    pos_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                rot_mat, _ = cv2.Rodrigues(rvec)
                quat = R.from_matrix(rot_mat).as_quat() 

                print(
                    f"\nMarker ID: {marker_id[0]}"
                    f"\nPosition (m): {tvec}"
                    f"\nQuaternion: {quat}"
                )

        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()