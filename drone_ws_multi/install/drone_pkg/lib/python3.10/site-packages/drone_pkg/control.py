import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from djitellopy import Tello
from djitellopy import TelloSwarm
import sys, select, termios, tty
from threading import Thread
import time
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

swarm = TelloSwarm.fromIps([
    "192.168.1.107",
    "192.168.1.102",
    "192.168.1.103",
    "192.168.1.108"
    #"192.168.1.105"
])
swarm.connect()

tello_takeoff = False
"""tello_hontai = []
for tello in swarm:
    tello_hontai.append(tello)"""

#target_pos = [[120, 0], [-60, 104], [-60, -104]] #3 drones
target_pos = [[75, 75], [-75, 75], [-75, -75], [75, -75]] #4 drones
#target_pos = [[120, 0], [37, 114], [-97, 71], [-97,- 71], [37, -114]] #5 drones

"""def uav_camera(tello, number):
    cv_bridge = CvBridge()
    tello.streamon()
    tello.set_video_direction(Tello.CAMERA_FORWARD)
    frame_read = tello.get_frame_read()
    img_publisher = node.create_publisher(CompressedImage, 'camera%s'%(str(number)), 1)
    while True:
        img = frame_read.frame
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
        img_publisher.publish(img_msg)"""    
def uav_control():
    global tello_takeoff, target_pos
    """for tello in tello_hontai:
        tello.enable_mission_pads()
        tello.set_mission_pad_detection_direction(0)"""
    for tello in swarm:
        tello.enable_mission_pads()
        tello.set_mission_pad_detection_direction(0)
    while True:
        key = getKey() #获取键值
        if key:
            print("key is :", key)
            if key == '1':
                swarm.takeoff()
                #time.sleep(20)
                tello_takeoff = True
            elif key == '2':
                swarm.land()
            elif key == 'w':
                swarm.move_forward(30)
            elif key == 's':
                swarm.move_back(30)
            elif key == 'a':
                swarm.move_left(30)
            elif key == 'd':
                swarm.move_right(30)
            elif key == 'e':
                swarm.rotate_clockwise(30)
            elif key == 'q':
                swarm.rotate_counter_clockwise(30)
            elif key == 'r':
                swarm.move_up(30)
            elif key == 'f':
                swarm.move_down(30)
            elif key == 'v':
                swarm.disable_mission_pads()
                """for tello in tello_hontai:
                    tello.disable_mission_pads()
                    swarm.disable_mission_pads()
                    #tello.streamoff()"""
                swarm.land()
                swarm.end()
                time.sleep(5)
                exit(0)
        
        if tello_takeoff:
            """swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(75, 75, 80, 50, 12))
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(-75, 75, 80, 50, 12))
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(-75, -75, 80, 50, 12))
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(75, -75, 80, 50, 12))"""
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[i%4][0], target_pos[i%4][1], 80, 50, 12))
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[(i+1)%4][0], target_pos[(i+1)%4][1], 80, 50, 12))
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[(i+2)%4][0], target_pos[(i+2)%4][1], 80, 50, 12))
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[(i+3)%4][0], target_pos[(i+3)%4][1], 80, 50, 12))
            #swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[i+4][0], target_pos[i+4][1], 80, 50, 12))

def main(args=None):
    rclpy.init(args=args)
    global node
    node = Node("control")
    """i = 1
    for tello in swarm:
        Thread(target=uav_camera, args=(tello, i,)).start()
        i += 1 """
    uav_control()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
