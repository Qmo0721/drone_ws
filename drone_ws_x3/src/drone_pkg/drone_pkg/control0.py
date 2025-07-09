import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from djitellopy import Tello
import sys, select, termios, tty
from threading import Thread
import time
#import serial
from ctypes import *
mylib = CDLL("/home/sunrise/drone_ws/canusb.so")
control = mylib.control
received_payload = b''
"""def setto(control):
    global bluetooth
    bluetooth.flushInput()
    ret = bluetooth.write(str.encode(control))
    time.sleep(0.01)"""
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
#tello.streamon()
#frame_read = tello.get_frame_read()

def uav_camera():
    cv_bridge = CvBridge()
    img_publisher = node.create_publisher(CompressedImage, 'camera0', 1)
    while True:
        img = frame_read.frame
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
        img_publisher.publish(img_msg)
def uav_control():
    """#cv_bridge = CvBridge()
    #img_publisher = node.create_publisher(CompressedImage, 'camera0', 10)
    ctr_msg = String()
    ctr_publisher = node.create_publisher(String, 'control', 1)
    while True:
        #img = frame_read.frame
        #img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "bgr8")
        #img_publisher.publish(img_msg)
        key = getKey() #获取键值
        if key:
            print("key is :", key)
            if key == '1':
                ctr_msg.data = "takeoff"
                ctr_publisher.publish(ctr_msg)
                tello.takeoff()
            elif key == '2':
    	        ctr_msg.data = "land"
    	        ctr_publisher.publish(ctr_msg)
    	        tello.land()
            elif key == 'w':
                #setto("<+00+10>")
                control(("1010010001003207").encode("utf-8"))
                ctr_msg.data = "move_forward"
                ctr_publisher.publish(ctr_msg)
                tello.move_forward(20)
            elif key == 's':
                #setto("<+00-10>")
                control(("1010000100013207").encode("utf-8"))
                ctr_msg.data = "move_back"
                ctr_publisher.publish(ctr_msg)
                tello.move_back(20)
            elif key == 'a':
                ctr_msg.data = "move_left"
                ctr_publisher.publish(ctr_msg)
                tello.move_left(20)
            elif key == 'd':
                ctr_msg.data = "move_right"
                ctr_publisher.publish(ctr_msg)
                tello.move_right(20)
            elif key == 'e':
                ctr_msg.data = "rotate_clockwise"
                ctr_publisher.publish(ctr_msg)
                tello.rotate_clockwise(10)
            elif key == 'q':
                ctr_msg.data = "rotate_counter_clockwise"
                ctr_publisher.publish(ctr_msg)
                tello.rotate_counter_clockwise(10)
            elif key == 'r':
                ctr_msg.data = "move_up"
                ctr_publisher.publish(ctr_msg)
                tello.move_up(20)
            elif key == 'f':
                ctr_msg.data = "move_down"
                ctr_publisher.publish(ctr_msg)
                tello.move_down(20)
            elif key == 'v':
                #setto("<+00+00>")
                control(("0000010001003207").encode("utf-8"))
                ctr_msg.data = "stop"
                ctr_publisher.publish(ctr_msg)
                tello.land()
                time.sleep(5)
                exit(0)
            #ctr_publisher.publish(ctr_msg)"""

def main(args=None):
    #global bluetooth
    #bluetooth = serial.Serial("/dev/rfcomm1",115200)
    rclpy.init(args=args)
    global node
    node = Node("control0")
    #Thread(target=uav_camera).start()
    uav_control()
    rclpy.spin(node)
    rclpy.shutdown()
    #bluetooth.close()
if __name__ == '__main__':
    main()
