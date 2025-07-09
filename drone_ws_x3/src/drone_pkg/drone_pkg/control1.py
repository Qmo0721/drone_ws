import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from djitellopy import Tello
import sys, select, termios, tty
from threading import Thread
import time
    
tello = Tello()
tello.connect()
tello.streamon()
frame_read = tello.get_frame_read()
def uav_camera():
    cv_bridge = CvBridge()
    img_publisher = node.create_publisher(CompressedImage, "camera1", 1)
    while True:
        img = frame_read.frame
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
        img_publisher.publish(img_msg)
def uav_control(msg):
    print(msg.data)
    if msg.data == "takeoff":
        tello.takeoff()
    elif msg.data == "land":
        tello.land()
    elif msg.data == "move_forward":
        tello.move_forward(20)
    elif msg.data == "move_back":
        tello.move_back(20)
    elif msg.data == "move_left":
        tello.move_left(20)
    elif msg.data == "move_right":
        tello.move_right(20)
    elif msg.data == "rotate_clockwise":
        tello.rotate_clockwise(10)
    elif msg.data == "rotate_counter_clockwise":
        tello.rotate_counter_clockwise(10)
    elif msg.data == "move_up":
        tello.move_up(20)
    elif msg.data == "move_down":
        tello.move_down(20)
    elif msg.data == "stop":
        tello.land()
        time.sleep(5)
        exit(0)

def main(args=None):
    rclpy.init(args=args)
    global node
    node = Node("control1")
    Thread(target=uav_camera).start()
    ctr_subscription = node.create_subscription(String, "control", uav_control, 1)
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
