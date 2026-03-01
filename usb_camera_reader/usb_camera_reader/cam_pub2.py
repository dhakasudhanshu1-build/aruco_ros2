import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('usb_camera')

        # Publisher topics
        self.image_pub = self.create_publisher(
            Image, '/camera/image_raw', 10)

        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info', 10)

        # OpenCV
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            raise RuntimeError("Camera not available")

        # Publish timer
        self.timer = self.create_timer(0.1, self.publish_frame)

        # Publish camera_info only once
        self.camera_info_sent = False

        self.get_logger().info("Camera Publisher started")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Publish Image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(img_msg)

        # Publish CameraInfo ONCE
        if not self.camera_info_sent:
            cam_info = CameraInfo()

            cam_info.width = 1280
            cam_info.height = 720
            cam_info.distortion_model = "plumb_bob"

            cam_info.k = [
                950.64077, 0.0, 666.86577,
                0.0, 947.49475, 365.92328,
                0.0, 0.0, 1.0
            ]

            cam_info.d = [
                -0.105338,
                 0.139434,
                 0.001233,
                 0.006295,
                 0.0
            ]

            self.camera_info_pub.publish(cam_info)
            self.camera_info_sent = True

        # Optional local preview
        cv2.imshow("USB Camera", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()