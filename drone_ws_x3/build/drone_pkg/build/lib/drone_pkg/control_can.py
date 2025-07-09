import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from djitellopy import Tello
import sys, select, termios, tty
import threading
import time
from ctypes import *
mylib = CDLL("/home/sunrise/drone_ws/canusb.so")
control = mylib.control
received_payload = b''
settings = termios.tcgetattr(sys.stdin) #获取键值初始化，读取终端相关属性
#获取键值函数
def getKey():
    global settings
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
tello = Tello()
tello.connect()
tello.streamon()
frame_read = tello.get_frame_read()

def camera():
    #global node
    cv_bridge = CvBridge()
    publisher = node.create_publisher(Image, 'camera', 10)
    while True:
        img = frame_read.frame
        msg = cv_bridge.cv2_to_imgmsg(img, "bgr8")
        publisher.publish(msg)
        key = getKey() #获取键值
        if key:
            print("key is :", key)
            if key == '1':
                tello.takeoff()
            elif key == '2':
                tello.land()
            elif key == 'w':
                control(("1010010001003207").encode("utf-8"))
                tello.move_forward(30)
            elif key == 's':
                control(("1010000100013207").encode("utf-8"))
                tello.move_back(30)
            elif key == 'a':
                tello.move_left(30)
            elif key == 'd':
                tello.move_right(30)
            elif key == 'e':
                tello.rotate_clockwise(30)
            elif key == 'q':
                tello.rotate_counter_clockwise(30)
            elif key == 'r':
                tello.move_up(30)
            elif key == 'f':
                tello.move_down(30)
            elif key == 'v':
                control(("0000010001003207").encode("utf-8"))
                tello.land()
                #time.sleep(3)
                #exit(0)

def main(args=None):
    rclpy.init(args=args)
    global node
    node = Node("control")
    camera()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
