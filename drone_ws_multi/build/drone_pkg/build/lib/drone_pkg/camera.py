import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('view')
        self.subscriber1 = self.create_subscription(CompressedImage, 'camera1', self.camera_callback1, 1)
        self.subscriber2 = self.create_subscription(CompressedImage, 'camera2', self.camera_callback2, 1)
        self.cv_bridge = CvBridge()
        print("init finish")
    
    def camera_callback1(self, msg):
        print("recv from /camera1")
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (720, 480))
        cv2.imshow("Camera1", img)
        cv2.waitKey(1)
    def camera_callback2(self, msg):
        print("recv from /camera2")
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (720, 480))
        cv2.imshow("Camera2", img)
        cv2.waitKey(1)
def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
