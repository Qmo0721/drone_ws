# Vehicle-Drone Coordination

The RDK X3 Master and Slave are connected to two drones via WiFi, respectively. The Master connects to the vehicle's lower-level controller via Bluetooth or a USBCAN analyzer. The Master and Slave are connected via Ethernet. When the Master controls Drone 0 and the vehicle, it uses the ROS2 Foxy publish-subscribe mechanism, and the Slave subscribes to the messages published by the Master to control Drone 1.

## Bluetooth Vehicle Control

### Code Overview

In the Master control code, the captured image data is published to the `camera0` topic for display by other programs. When a keyboard key is pressed, messages are sent to the Slave via ROS2 over the local network and to the vehicle via Bluetooth, while simultaneously controlling the drone's movement. The Master control code is as follows:
```python
import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import CompressedImage  
from std_msgs.msg import String  
from cv_bridge import CvBridge  
from djitellopy import Tello  
import sys, select, termios, tty  
from threading import Thread  
import time  
import serial  
def setto(control):  
    global bluetooth  
    bluetooth.flushInput()  
    ret = bluetooth.write(str.encode(control))  *# Send Bluetooth message*  
    time.sleep(0.01)  
settings = termios.tcgetattr(sys.stdin)  *# Key value initialization, read terminal attributes*  
*# Key value function*  
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

tello = Tello()  *# Initialize Tello*  
tello.connect()  *# Connect to the drone*  
tello.streamon()  *# Enable video stream*  
frame_read = tello.get_frame_read()  

def uav_camera():  
    cv_bridge = CvBridge()  
    img_publisher = node.create_publisher(CompressedImage, 'camera0', 1)  *# Publish CompressedImage type compressed image messages*  
    while True:  
        img = frame_read.frame  *# Capture a video frame*  
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")  
        img_publisher.publish(img_msg)  
def uav_control():  
    ctr_msg = String()  
    ctr_publisher = node.create_publisher(String, 'control', 1)  *# Publish String type drone control messages to Slave*  
    while True:  
        key = getKey()  *# Get key value*  
        if key:  
            print("key is :", key)  
            if key == '1':  *# Takeoff*  
                ...

def main(args=None):  
    global bluetooth  
    bluetooth = serial.Serial("/dev/rfcomm1",115200)  *# Establish Bluetooth connection*  
    rclpy.init(args=args)  
    global node  
    node = Node("control0")  
    Thread(target=uav_camera).start()  *# Start video stream display in a separate thread to prevent interruption during drone movement*  
    uav_control()  
    rclpy.spin(node)  
    rclpy.shutdown()  
    bluetooth.close()  *# Close Bluetooth connection*  
if __name__ == '__main__':  
    main()  
```
The Master image display code receives image data from the `camera0` topic and displays it via an OpenCV window. The Master image display code is as follows:  
```python
import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import CompressedImage  
from cv_bridge import CvBridge  
import cv2  
import numpy as np  

class CameraSubscriber(Node):  
    def __init__(self):  
        super().__init__('view')  
        self.subscriber = self.create_subscription(CompressedImage, 'camera0', self.camera_callback, 1)  *# Receive CompressedImage type compressed image messages*  
        self.cv_bridge = CvBridge()  
        print("init finish")  

    def camera_callback(self, msg):  
        print("recv from /camera")  
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)  
        img = cv2.resize(img, (720, 480))  
        cv2.imshow("Camera", img)  *# Display video stream in cv2 window*  
        cv2.waitKey(1)  

def main(args=None):  
    rclpy.init(args=args)  
    camera_subscriber = CameraSubscriber()  
    rclpy.spin(camera_subscriber)  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()  
```
The framework for the Master program is as follows:  

The Slave control code publishes the captured image data to the `camera1` topic for display by other programs. Upon receiving messages from the Master via ROS2 over the local network, it controls the drone's movement. The Slave control code is as follows:  
```python
import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import CompressedImage  
from std_msgs.msg import String  
from cv_bridge import CvBridge  
from djitellopy import Tello  
import sys, select, termios, tty  
from threading import Thread  
import time  

tello = Tello()  *# Initialize Tello*  
tello.connect()  *# Connect to the drone*  
tello.streamon()  *# Enable video stream*  
frame_read = tello.get_frame_read()  
def uav_camera():  
    cv_bridge = CvBridge()  
    img_publisher = node.create_publisher(CompressedImage, "camera1", 1)  *# Publish CompressedImage type compressed image messages*  
    while True:  
        img = frame_read.frame  
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")  
        img_publisher.publish(img_msg)  
def uav_control(msg):  
    print(msg.data)  
    if msg.data == "takeoff":  *# Takeoff*  
        tello.takeoff()  
    elif msg.data == "land":  *# Land*  
        tello.land()  
    ...

def main(args=None):  
    rclpy.init(args=args)  
    global node  
    node = Node("control1")  
    Thread(target=uav_camera).start()  *# Start video stream display in a separate thread to prevent interruption during drone movement*  
    ctr_subscription = node.create_subscription(String, "control", uav_control, 1)  *# Receive String type drone control messages from Master*  
    rclpy.spin(node)  
    rclpy.shutdown()  
if __name__ == '__main__':  
    main()  
```
The Slave image display code receives image data from the `camera1` topic and displays it via an OpenCV window. The Slave image display code is as follows:  
```python
import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import CompressedImage  
from cv_bridge import CvBridge  
import cv2  
import numpy as np  

class CameraSubscriber(Node):  
    def __init__(self):  
        super().__init__('view')  
        self.subscriber = self.create_subscription(CompressedImage, 'camera1', self.camera_callback, 1)  *# Receive CompressedImage type compressed image messages*  
        self.cv_bridge = CvBridge()  
        print("init finish")  

    def camera_callback(self, msg):  
        print("recv from /camera")  
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)  
        img = cv2.resize(img, (720, 480))  
        cv2.imshow("Camera", img)  *# Display video stream in cv2 window*  
        cv2.waitKey(1)  

def main(args=None):  
    rclpy.init(args=args)  
    camera_subscriber = CameraSubscriber()  
    rclpy.spin(camera_subscriber)  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()  
```
The framework for the Slave program is as follows:  

### Launching the Program  

The Bluetooth address needs to be entered in the Master terminal by typing `bluetoothctl`, then entering `scan on` to view the Bluetooth module address. Once the address is found, use the `pair address` command to pair, with the pairing password being `1234`. Modify the Bluetooth address in `connect_bluetooth.sh` to the one you want to pair with.  

In `drone_ws`, create a new terminal and run the following command to connect to Bluetooth:  
```bash
./connect_bluetooth.sh  
```
After connecting the Master to the drone's WiFi, create two new terminals in `drone_ws` and run the following commands to control the drone and vehicle via the keyboard:  
```bash
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws control0  
```
```bash
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws view0  
```
After connecting the Slave to the drone's WiFi, create two new terminals in `drone_ws` and run the following commands to control the drone and vehicle via the keyboard:  
```bash
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws control1  

sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws view1  
```
## CAN Vehicle Control  

### Code Overview  

In the Master control code, the captured image data is published to the `camera` topic for display by other programs. When a keyboard key is pressed, messages are sent to the Slave via ROS2 over the local network, and the `control` function from the `canusb.so` file compiled under `drone_ws` is called to send CAN messages to the vehicle while simultaneously controlling the drone's movement. The Master control code is as follows:  
```python
import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import CompressedImage  
from std_msgs.msg import String  
from cv_bridge import CvBridge  
from djitellopy import Tello  
import sys, select, termios, tty  
from threading import Thread  
import time  
from ctypes import *  
mylib = CDLL("/home/sunrise/drone_ws/canusb.so")  *# Load dynamic link library*  
control = mylib.control  *# Call function from dynamic link library*  
received_payload = b''  
settings = termios.tcgetattr(sys.stdin)  *# Key value initialization, read terminal attributes*  
*# Key value function*  
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

tello = Tello()  *# Initialize Tello*  
tello.connect()  *# Connect to the drone*  
tello.streamon()  *# Enable video stream*  
frame_read = tello.get_frame_read()  

def uav_camera():  
    cv_bridge = CvBridge()  
    img_publisher = node.create_publisher(CompressedImage, 'camera0', 1)  *# Publish CompressedImage type compressed image messages*  
    while True:  
        img = frame_read.frame  
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")  
        img_publisher.publish(img_msg)  
def uav_control():  
    ctr_msg = String()  
    ctr_publisher = node.create_publisher(String, 'control', 1)  *# Publish String type drone control messages*  
    while True:  
        key = getKey()  *# Get key value*  
        if key:  
            print("key is :", key)  
            if key == '1':  *# Takeoff*  
                ...

def main(args=None):  
    rclpy.init(args=args)  
    global node  
    node = Node("control0")  
    Thread(target=uav_camera).start()  *# Start video stream display in a separate thread to prevent interruption during drone movement*  
    uav_control()  
    rclpy.spin(node)  
    rclpy.shutdown()  
if __name__ == '__main__':  
    main()  
```
The Master image display code receives image data from the `camera0` topic and displays it via an OpenCV window. The image display code is as follows:  
```python
import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import CompressedImage  
from cv_bridge import CvBridge  
import cv2  
import numpy as np  

class CameraSubscriber(Node):  
    def __init__(self):  
        super().__init__('view')  
        self.subscriber = self.create_subscription(CompressedImage, 'camera0', self.camera_callback, 1)  *# Receive CompressedImage type compressed image messages*  
        self.cv_bridge = CvBridge()  
        print("init finish")  

    def camera_callback(self, msg):  
        print("recv from /camera")  
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)  
        img = cv2.resize(img, (720, 480))  
        cv2.imshow("Camera", img)  *# Display video stream in cv2 window*  
        cv2.waitKey(1)  

def main(args=None):  
    rclpy.init(args=args)  
    camera_subscriber = CameraSubscriber()  
    rclpy.spin(camera_subscriber)  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()  
```
The framework for the Master program is as follows:  

The Slave control code publishes the captured image data to the `camera1` topic for display by other programs. Upon receiving messages from the Master via ROS2 over the local network, it controls the drone's movement. The Slave control code is as follows:  
```python
import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import CompressedImage  
from std_msgs.msg import String  
from cv_bridge import CvBridge  
from djitellopy import Tello  
import sys, select, termios, tty  
from threading import Thread  
import time  

tello = Tello()  *# Initialize Tello*  
tello.connect()  *# Connect to the drone*  
tello.streamon()  *# Enable video stream*  
frame_read = tello.get_frame_read()  
def uav_camera():  
    cv_bridge = CvBridge()  
    img_publisher = node.create_publisher(CompressedImage, "camera1", 1)  *# Publish CompressedImage type compressed image messages*  
    while True:  
        img = frame_read.frame  
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")  
        img_publisher.publish(img_msg)  
def uav_control(msg):  
    print(msg.data)  
    if msg.data == "takeoff":  *# Takeoff*  
        tello.takeoff()  
    elif msg.data == "land":  *# Land*  
        tello.land()  
    ...

def main(args=None):  
    rclpy.init(args=args)  
    global node  
    node = Node("control1")  
    Thread(target=uav_camera).start()  *# Start video stream display in a separate thread to prevent interruption during drone movement*  
    ctr_subscription = node.create_subscription(String, "control", uav_control, 1)  *# Receive String type drone control messages*  
    rclpy.spin(node)  
    rclpy.shutdown()  
if __name__ == '__main__':  
    main()  
```
The Slave image display code receives image data from the `camera1` topic and displays it via an OpenCV window. The Slave image display code is as follows:  
```python
import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import CompressedImage  
from cv_bridge import CvBridge  
import cv2  
import numpy as np  

class CameraSubscriber(Node):  
    def __init__(self):  
        super().__init__('view')  
        self.subscriber = self.create_subscription(CompressedImage, 'camera1', self.camera_callback, 1)  *# Receive CompressedImage type compressed image messages*  
        self.cv_bridge = CvBridge()  
        print("init finish")  

    def camera_callback(self, msg):  
        print("recv from /camera")  
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)  
        img = cv2.resize(img, (720, 480))  
        cv2.imshow("Camera", img)  *# Display video stream in cv2 window*  
        cv2.waitKey(1)  

def main(args=None):  
    rclpy.init(args=args)  
    camera_subscriber = CameraSubscriber()  
    rclpy.spin(camera_subscriber)  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()  
```
The framework for the Slave program is as follows:  

### Launching the Program  

The Master's USB port is connected to the vehicle's CAN interface via a USBCAN analyzer.  

After connecting the Master to the drone's WiFi, create two new terminals in `drone_ws` and run the following commands to control the drone and vehicle via the keyboard:  
```bash
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws control0  

sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws view0  
```
After connecting the Slave to the drone's WiFi, create two new terminals in `drone_ws` and run the following commands to control the drone and vehicle via the keyboard:  
```bash
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws control1  

sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws view1  
```
# Experimental Steps  

1_1. Connect to the vehicle (Bluetooth): In the Master terminal, enter `bluetoothctl`, then enter `scan on` to view the Bluetooth module address. Once the address is found, use the `pair address` command to pair, with the pairing password being `1234`. Modify the Bluetooth address in `connect_bluetooth.sh` to the one you want to pair with. In `drone_ws`, create a new terminal and run the command `./connect_bluetooth` to connect to Bluetooth.  

1_2. Connect to the vehicle (CAN): Connect the Master's USB port to the vehicle's CAN interface via a USBCAN analyzer.  

2. Launch the Master control program: On the Master, navigate to the `drone_ws` folder, open a terminal, and enter the following commands:
```bash
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws control0
```

3. Launch the Master image display program: On the Master, navigate to the `drone_ws` folder, open a terminal, and enter the following commands:
```bash
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws view0
```

4. Launch the Slave control program: On the Slave, navigate to the `drone_ws` folder, open a terminal, and enter the following commands:
```bash 
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws control1
```

5. Launch the Slave image display program: On the Slave, navigate to the `drone_ws` folder, open a terminal, and enter the following commands:
```bash
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws view1
```

6. Control the drones and vehicle: Use the keyboard in the terminal running the Master control program to control the two drones and the vehicle.  

# Multi-Drone Formation  

The RDK development board and drones are connected to the same router's WiFi. After successful connection, the drone's propellers will automatically start spinning. Once the RDK development board is successfully connected, you can view the IP addresses of each device in the router management page. The RDK development board sends commands to the drone formation via WiFi for control.  

## Restoring the Expansion Board to Default Settings  

Connect the expansion board's microUSB to a computer.  

Go to https://mindplus.cc/, download and install Mind+.  

Open Mind+, select Upload Mode, then select Connect Device, choose the Extensions tab at the bottom left, and select RoboMaster TT (ESP32) before returning.  

Now, select Restore Device to Default Settings in the device bar until the bottom right displays the data. If the serial port driver is not installed on the computer, you can choose One-Click Installation.  

Connect the expansion board's USB to the drone, power on the drone, and switch the expansion board's switch to the lower position (Single Mode).  

In Mind+, select Real-Time Mode, click the Extensions tab at the bottom left, and choose RoboMaster TT (Single) under the Functional Modules section.  

Then click Back, followed by Functional Modules and the exclamation mark.  

Connect the computer to the drone's network via WiFi, then connect to the drone. If the connection fails, refresh and try again until successful.  

Drag the module for switching to Station Mode into the middle blank area, and modify it to the prepared WiFi router hotspot name and password.  

Then, connect the computer to the same WiFi, double-click the modified module, and you will see the message "OK, drone will reboot in 3s."  

Switch the drone's expansion board switch to the upper position. After a while, when the drone automatically connects to WiFi, the propellers will start spinning.  

In the router management page, locate the connected drones to find each drone's corresponding IP address.  

## 2.2 Code Overview  

Use the `TelloSwarm` class from the `djitellopy` library to connect to four drones (specify their IP addresses; for accuracy, check the router management page). Enable the mission pad positioning function and set the bottom camera to detect mission pads.  

First, place the four drones in the designated positions on the flight map in order. Then, press the keyboard key `1` to trigger the swarm takeoff. After takeoff, the four drones will fly in a loop according to the preset coordinates `target_pos` (four vertices of a 75 cm square). Each drone will fly to the next target point in sequence (e.g., Drone1: A→B→C→D→A), while using `go_xyz_speed_mid` for high-precision positioning flight (relying on the flight map's coordinate feedback).  

Create a new ROS2 workspace `drone_ws` and package `drone_pkg`. Under `drone_pkg`, create a new program `control.py` with the following content:  
```python
import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import CompressedImage  
from cv_bridge import CvBridge  
from djitellopy import Tello  
from djitellopy import TelloSwarm  
import sys, select, termios, tty  
from threading import Thread  
import time  
settings = termios.tcgetattr(sys.stdin)  *# Key value initialization, read terminal attributes*  
*# Key value function*  
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
    "192.168.1.101",  
    "192.168.1.102",  
    "192.168.1.103",  
    "192.168.1.104"  
])  
swarm.connect()  
tello_takeoff = False  
target_pos = [[75, 75], [-75, 75], [-75, -75], [75, -75]]  *# 4 drones*  

def uav_control():  
    global tello_takeoff, target_pos  
    for tello in swarm:  
        tello.enable_mission_pads()  
        tello.set_mission_pad_detection_direction(0)  
    while True:  
        key = getKey()  *# Get key value*  
        if key:  
            print("key is :", key)  
            if key == '1':  
                swarm.takeoff()  
                *#time.sleep(20)*  
                tello_takeoff = True  
            elif key == '2':  
                swarm.land()  
            ...

        *# Fly around four points*  
        if tello_takeoff:  
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[i%4][0], target_pos[i%4][1], 80, 50, 12))  
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[(i+1)%4][0], target_pos[(i+1)%4][1], 80, 50, 12))  
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[(i+2)%4][0], target_pos[(i+2)%4][1], 80, 50, 12))  
            swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[(i+3)%4][0], target_pos[(i+3)%4][1], 80, 50, 12))  

def main(args=None):  
    rclpy.init(args=args)  
    global node  
    node = Node("control")  
    uav_control()  
    rclpy.spin(node)  
    rclpy.shutdown()  
if __name__ == '__main__':  
    main()  
```
## 2.3 Launching the Program  

Run the following command to install dependencies:  
```bash
pip install djitellopy  
```
Connect the RDK development board to the same WiFi used during the expansion board's initial setup. Switch the drone's expansion board switch to the upper position, power on the drone, and wait for the propellers to start spinning automatically.  

In the `drone_ws` workspace, run the following commands to launch the program. (For accuracy, check each drone's IP address in the router management page and modify the code accordingly.)  
```bash
colcon build  
source install/setup.bash  
ros2 run drone_pkg control  
```
Once all four drones are successfully connected, press the `1` key and wait for the drones to take off. They will then automatically fly toward the target points.  
