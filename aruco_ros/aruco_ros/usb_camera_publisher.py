import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('usb_camera_publisher')

        # Parameters
        self.declare_parameter('camera_id', 0)
        camera_id = self.get_parameter('camera_id').value

        # Topics
        self.image_topic = '/camera/image_raw'
        self.camera_info_topic = '/camera/camera_info'

        # Publishers
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.publish_frame)

        # OpenCV
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(camera_id)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            raise SystemExit

        self.camera_info_sent = False
        self.get_logger().info("USB Camera Publisher started")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        height, width = frame.shape[:2]

        # Publish Image
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(image_msg)

        # Publish CameraInfo ONCE
        if not self.camera_info_sent:
            cam_info = CameraInfo()
            cam_info.width = 1280
            cam_info.height = 720

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

            cam_info.distortion_model = "plumb_bob"

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