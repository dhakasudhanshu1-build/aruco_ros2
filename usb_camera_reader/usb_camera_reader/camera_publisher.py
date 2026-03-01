import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('usb_camera_publisher')

        
        self.declare_parameter('camera_id', 0)  
        camera_id = self.get_parameter('camera_id').value

        self.topic_name = '/usb_camera/image_raw'
        self.camera_id = '/dev/video' + str(camera_id)
        self.publisher_ = self.create_publisher(Image, self.topic_name, 10)
        self.get_frame_timer = self.create_timer(0.1, self.get_image_frame)
        self.image_elaboration_timer = self.create_timer( 0.1, self.image_elaboration)

        self.bridge = CvBridge()      
        self.frame = []    
        self.img_ready = False
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video device with ID {camera_id}")
            raise SystemExit        
    
    def get_image_frame(self):
        ret, self.frame = self.cap.read()
        if ret:
            height, width = self.frame.shape[:2]
            new_size = (width // 3, height // 3)
            resized_frame = cv2.resize(self.frame, new_size)

            # Step 2: Convert the resized image to grayscale (black and white)
            gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow('frame', gray_frame)
            cv2.waitKey(1)
            self.img_ready = True
            
    def image_elaboration(self):
        if( self.img_ready == True):
            frame = self.frame
        
            height, width, _ = frame.shape
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.6
            font_color = (255, 255, 255)  
            font_thickness = 3
            resolution_text = f"Resolution: {width}x{height}"
            position_device = (10, 30) 
            position_topic = (10, 60)
            position_res = (10, 90) 
            
            cv2.putText(frame, "Devce: " + self.camera_id, position_device, font, font_scale, font_color, font_thickness)
            cv2.putText(frame, "Topic: " + self.topic_name, position_topic, font, font_scale, font_color, font_thickness)
            cv2.putText(frame, resolution_text, position_res, font, font_scale, font_color, font_thickness)

      
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()
