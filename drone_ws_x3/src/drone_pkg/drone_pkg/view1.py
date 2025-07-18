import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):
    """def __init__(self):
        super().__init__('view')
        self.subscriber = self.create_subscription(CompressedImage, 'camera1', self.camera_callback, 1)
        self.cv_bridge = CvBridge()
        print("init finish")
    
    def camera_callback(self, msg):
        print("recv from /camera")
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.resize(img, (720, 480))
        cv2.imshow("Camera", img)
        cv2.waitKey(1)"""
    
def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
