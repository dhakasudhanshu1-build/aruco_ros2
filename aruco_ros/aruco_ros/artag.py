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

        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback, 
            10
        )
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )


        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.got_camera_info = False
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.get_logger().info("Aruco Marker Node has been started.")


    def camera_info_callback(self, msg: CameraInfo):
        # Retrieve the camera calibration parameters from CameraInfo message
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.got_camera_info = True
        self.get_logger().info("Camera calibration received.")


    def image_callback(self, msg: Image):
        if not self.got_camera_info:
            self.get_logger().warning("Waiting for camera info...")
            return

        # Convert the ROS Image message to a CV2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale (Aruco detection works on grayscale images)
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect Aruco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, 0.2, self.camera_matrix, self.dist_coeffs)
            

            # Draw axis for each marker
            for i, rvec, tvec in zip(ids, rvecs, tvecs):
                
                cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                marker_position = f"Position: {tvec[0][0]:.2f}, {tvec[0][1]:.2f}, {tvec[0][2]:.2f}"
                cv2.putText(cv_image, marker_position, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                r = R.from_euler('xyz', rvec, degrees=False)  # 'xyz' corresponds to roll, pitch, yaw
                quaternion = r.as_quat()  # Returns [x, y, z, w]
                print ("Marker ID:", i, "\nPosition: ", tvec, "\nOrientation: ", quaternion)

        # Show the image with Aruco markers
        cv2.imshow('Aruco Markers', cv_image)
        cv2.waitKey(1)  # For image display refresh

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

