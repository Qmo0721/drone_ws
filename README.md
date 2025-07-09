# Vehicle-Drone Coordination

The RDK X3 Master and Slave are connected to two drones via WiFi, respectively. The Master connects to the vehicle's lower-level controller via Bluetooth or a USBCAN analyzer. The Master and Slave are connected via Ethernet. When the Master controls Drone 0 and the vehicle, it uses the ROS2 Foxy publish-subscribe mechanism, and the Slave subscribes to the messages published by the Master to control Drone 1.

## Bluetooth Vehicle Control

### Code Overview

In the Master control code, the captured image data is published to the `camera0` topic for display by other programs. When a keyboard key is pressed, messages are sent to the Slave via ROS2 over the local network and to the vehicle via Bluetooth, while simultaneously controlling the drone's movement. The Master control code is as follows:

1.  import rclpy  
2.  from rclpy.node import Node  
3.  from sensor_msgs.msg import CompressedImage  
4.  from std_msgs.msg import String  
5.  from cv_bridge import CvBridge  
6.  from djitellopy import Tello  
7.  import sys, select, termios, tty  
8.  from threading import Thread  
9.  import time  
10. import serial  
11. def setto(control):  
12.     global bluetooth  
13.     bluetooth.flushInput()  
14.     ret = bluetooth.write(str.encode(control))  *# Send Bluetooth message*  
15.     time.sleep(0.01)  
16. settings = termios.tcgetattr(sys.stdin)  *# Key value initialization, read terminal attributes*  
17. *# Key value function*  
18. def getKey():  
19.     global settings  
20.     tty.setraw(sys.stdin.fileno())  
21.     rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  
22.     if rlist:  
23.         key = sys.stdin.read(1)  
24.     else:  
25.         key = ''  

27.     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  
28.     return key  

30. tello = Tello()  *# Initialize Tello*  
31. tello.connect()  *# Connect to the drone*  
32. tello.streamon()  *# Enable video stream*  
33. frame_read = tello.get_frame_read()  

35. def uav_camera():  
36.     cv_bridge = CvBridge()  
37.     img_publisher = node.create_publisher(CompressedImage, 'camera0', 1)  *# Publish CompressedImage type compressed image messages*  
38.     while True:  
39.         img = frame_read.frame  *# Capture a video frame*  
40.         img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")  
41.         img_publisher.publish(img_msg)  
42. def uav_control():  
43.     ctr_msg = String()  
44.     ctr_publisher = node.create_publisher(String, 'control', 1)  *# Publish String type drone control messages to Slave*  
45.     while True:  
46.         key = getKey()  *# Get key value*  
47.         if key:  
48.             print("key is :", key)  
49.             if key == '1':  *# Takeoff*  
50.                 ctr_msg.data = "takeoff"  
51.                 ctr_publisher.publish(ctr_msg)  
52.                 tello.takeoff()  
53.             elif key == '2':  *# Land*  
54.              ctr_msg.data = "land"  
55.              ctr_publisher.publish(ctr_msg)  
56.              tello.land()  
57.             elif key == 'w':  *# Move forward*  
58.                 setto("<+00+10>")  
59.                 ctr_msg.data = "move_forward"  
60.                 ctr_publisher.publish(ctr_msg)  
61.                 tello.move_forward(20)  
62.             elif key == 's':  *# Move backward*  
63.                 setto("<+00-10>")  
64.                 ctr_msg.data = "move_back"  
65.                 ctr_publisher.publish(ctr_msg)  
66.                 tello.move_back(20)  
67.             elif key == 'a':  *# Move left*  
68.                 ctr_msg.data = "move_left"  
69.                 ctr_publisher.publish(ctr_msg)  
70.                 tello.move_left(20)  
71.             elif key == 'd':  *# Move right*  
72.                 ctr_msg.data = "move_right"  
73.                 ctr_publisher.publish(ctr_msg)  
74.                 tello.move_right(20)  
75.             elif key == 'e':  *# Rotate clockwise*  
76.                 ctr_msg.data = "rotate_clockwise"  
77.                 ctr_publisher.publish(ctr_msg)  
78.                 tello.rotate_clockwise(10)  
79.             elif key == 'q':  *# Rotate counter-clockwise*  
80.                 ctr_msg.data = "rotate_counter_clockwise"  
81.                 ctr_publisher.publish(ctr_msg)  
82.                 tello.rotate_counter_clockwise(10)  
83.             elif key == 'r':  *# Move up*  
84.                 ctr_msg.data = "move_up"  
85.                 ctr_publisher.publish(ctr_msg)  
86.                 tello.move_up(20)  
87.             elif key == 'f':  *# Move down*  
88.                 ctr_msg.data = "move_down"  
89.                 ctr_publisher.publish(ctr_msg)  
90.                 tello.move_down(20)  
91.             elif key == 'v':  *# Stop*  
92.                 setto("<+00+00>")  
93.                 ctr_msg.data = "stop"  
94.                 ctr_publisher.publish(ctr_msg)  
95.                 tello.land()  
96.                 time.sleep(5)  
97.                 exit(0)  

99. def main(args=None):  
100.    global bluetooth  
101.    bluetooth = serial.Serial("/dev/rfcomm1",115200)  *# Establish Bluetooth connection*  
102.    rclpy.init(args=args)  
103.    global node  
104.    node = Node("control0")  
105.    Thread(target=uav_camera).start()  *# Start video stream display in a separate thread to prevent interruption during drone movement*  
106.    uav_control()  
107.    rclpy.spin(node)  
108.    rclpy.shutdown()  
109.    bluetooth.close()  *# Close Bluetooth connection*  
110.if __name__ == '__main__':  
111.    main()  

The Master image display code receives image data from the `camera0` topic and displays it via an OpenCV window. The Master image display code is as follows:  

1.  import rclpy  
2.  from rclpy.node import Node  
3.  from sensor_msgs.msg import CompressedImage  
4.  from cv_bridge import CvBridge  
5.  import cv2  
6.  import numpy as np  

8.  class CameraSubscriber(Node):  
9.      def __init__(self):  
10.         super().__init__('view')  
11.         self.subscriber = self.create_subscription(CompressedImage, 'camera0', self.camera_callback, 1)  *# Receive CompressedImage type compressed image messages*  
12.         self.cv_bridge = CvBridge()  
13.         print("init finish")  

15.     def camera_callback(self, msg):  
16.         print("recv from /camera")  
17.         img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)  
18.         img = cv2.resize(img, (720, 480))  
19.         cv2.imshow("Camera", img)  *# Display video stream in cv2 window*  
20.         cv2.waitKey(1)  

22. def main(args=None):  
23.     rclpy.init(args=args)  
24.     camera_subscriber = CameraSubscriber()  
25.     rclpy.spin(camera_subscriber)  
26.     rclpy.shutdown()  

28. if __name__ == '__main__':  
29.     main()  

The framework for the Master program is as follows:  

The Slave control code publishes the captured image data to the `camera1` topic for display by other programs. Upon receiving messages from the Master via ROS2 over the local network, it controls the drone's movement. The Slave control code is as follows:  

1.  import rclpy  
2.  from rclpy.node import Node  
3.  from sensor_msgs.msg import CompressedImage  
4.  from std_msgs.msg import String  
5.  from cv_bridge import CvBridge  
6.  from djitellopy import Tello  
7.  import sys, select, termios, tty  
8.  from threading import Thread  
9.  import time  

11. tello = Tello()  *# Initialize Tello*  
12. tello.connect()  *# Connect to the drone*  
13. tello.streamon()  *# Enable video stream*  
14. frame_read = tello.get_frame_read()  
15. def uav_camera():  
16.     cv_bridge = CvBridge()  
17.     img_publisher = node.create_publisher(CompressedImage, "camera1", 1)  *# Publish CompressedImage type compressed image messages*  
18.     while True:  
19.         img = frame_read.frame  
20.         img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")  
21.         img_publisher.publish(img_msg)  
22. def uav_control(msg):  
23.     print(msg.data)  
24.     if msg.data == "takeoff":  *# Takeoff*  
25.         tello.takeoff()  
26.     elif msg.data == "land":  *# Land*  
27.         tello.land()  
28.     elif msg.data == "move_forward":  *# Move forward*  
29.         tello.move_forward(20)  
30.     elif msg.data == "move_back":  *# Move backward*  
31.         tello.move_back(20)  
32.     elif msg.data == "move_left":  *# Move left*  
33.         tello.move_left(20)  
34.     elif msg.data == "move_right":  *# Move right*  
35.         tello.move_right(20)  
36.     elif msg.data == "rotate_clockwise":  *# Rotate clockwise*  
37.         tello.rotate_clockwise(10)  
38.     elif msg.data == "rotate_counter_clockwise":  *# Rotate counter-clockwise*  
39.         tello.rotate_counter_clockwise(10)  
40.     elif msg.data == "move_up":  *# Move up*  
41.         tello.move_up(20)  
42.     elif msg.data == "move_down":  *# Move down*  
43.         tello.move_down(20)  
44.     elif msg.data == "stop":  *# Stop*  
45.         tello.land()  
46.         time.sleep(5)  
47.         exit(0)  

49. def main(args=None):  
50.     rclpy.init(args=args)  
51.     global node  
52.     node = Node("control1")  
53.     Thread(target=uav_camera).start()  *# Start video stream display in a separate thread to prevent interruption during drone movement*  
54.     ctr_subscription = node.create_subscription(String, "control", uav_control, 1)  *# Receive String type drone control messages from Master*  
55.     rclpy.spin(node)  
56.     rclpy.shutdown()  
57. if __name__ == '__main__':  
58.     main()  

The Slave image display code receives image data from the `camera1` topic and displays it via an OpenCV window. The Slave image display code is as follows:  

1.  import rclpy  
2.  from rclpy.node import Node  
3.  from sensor_msgs.msg import CompressedImage  
4.  from cv_bridge import CvBridge  
5.  import cv2  
6.  import numpy as np  

8.  class CameraSubscriber(Node):  
9.      def __init__(self):  
10.         super().__init__('view')  
11.         self.subscriber = self.create_subscription(CompressedImage, 'camera1', self.camera_callback, 1)  *# Receive CompressedImage type compressed image messages*  
12.         self.cv_bridge = CvBridge()  
13.         print("init finish")  

15.     def camera_callback(self, msg):  
16.         print("recv from /camera")  
17.         img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)  
18.         img = cv2.resize(img, (720, 480))  
19.         cv2.imshow("Camera", img)  *# Display video stream in cv2 window*  
20.         cv2.waitKey(1)  

22. def main(args=None):  
23.     rclpy.init(args=args)  
24.     camera_subscriber = CameraSubscriber()  
25.     rclpy.spin(camera_subscriber)  
26.     rclpy.shutdown()  

28. if __name__ == '__main__':  
29.     main()  

The framework for the Slave program is as follows:  

### Launching the Program  

The Bluetooth address needs to be entered in the Master terminal by typing `bluetoothctl`, then entering `scan on` to view the Bluetooth module address. Once the address is found, use the `pair address` command to pair, with the pairing password being `1234`. Modify the Bluetooth address in `connect_bluetooth.sh` to the one you want to pair with.  

In `drone_ws`, create a new terminal and run the following command to connect to Bluetooth:  

1.  ./connect_bluetooth.sh  

After connecting the Master to the drone's WiFi, create two new terminals in `drone_ws` and run the following commands to control the drone and vehicle via the keyboard:  

1.  sudo -s  
2.  source /opt/tros/setup.bash  
3.  source install/setup.bash  
4.  ros2 run drone_ws control0  

1.  sudo -s  
2.  source /opt/tros/setup.bash  
3.  source install/setup.bash  
4.  ros2 run drone_ws view0  

After connecting the Slave to the drone's WiFi, create two new terminals in `drone_ws` and run the following commands to control the drone and vehicle via the keyboard:  

1.  sudo -s  
2.  source /opt/tros/setup.bash  
3.  source install/setup.bash  
4.  ros2 run drone_ws control1  

1.  sudo -s  
2.  source /opt/tros/setup.bash  
3.  source install/setup.bash  
4.  ros2 run drone_ws view1  

## CAN Vehicle Control  

### Code Overview  

In the Master control code, the captured image data is published to the `camera` topic for display by other programs. When a keyboard key is pressed, messages are sent to the Slave via ROS2 over the local network, and the `control` function from the `canusb.so` file compiled under `drone_ws` is called to send CAN messages to the vehicle while simultaneously controlling the drone's movement. The Master control code is as follows:  

1.  import rclpy  
2.  from rclpy.node import Node  
3.  from sensor_msgs.msg import CompressedImage  
4.  from std_msgs.msg import String  
5.  from cv_bridge import CvBridge  
6.  from djitellopy import Tello  
7.  import sys, select, termios, tty  
8.  from threading import Thread  
9.  import time  
10. from ctypes import *  
11. mylib = CDLL("/home/sunrise/drone_ws/canusb.so")  *# Load dynamic link library*  
12. control = mylib.control  *# Call function from dynamic link library*  
13. received_payload = b''  
14. settings = termios.tcgetattr(sys.stdin)  *# Key value initialization, read terminal attributes*  
15. *# Key value function*  
16. def getKey():  
17.     global settings  
18.     tty.setraw(sys.stdin.fileno())  
19.     rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  
20.     if rlist:  
21.         key = sys.stdin.read(1)  
22.     else:  
23.         key = ''  

25.     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  
26.     return key  

28. tello = Tello()  *# Initialize Tello*  
29. tello.connect()  *# Connect to the drone*  
30. tello.streamon()  *# Enable video stream*  
31. frame_read = tello.get_frame_read()  

33. def uav_camera():  
34.     cv_bridge = CvBridge()  
35.     img_publisher = node.create_publisher(CompressedImage, 'camera0', 1)  *# Publish CompressedImage type compressed image messages*  
36.     while True:  
37.         img = frame_read.frame  
38.         img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")  
39.         img_publisher.publish(img_msg)  
40. def uav_control():  
41.     ctr_msg = String()  
42.     ctr_publisher = node.create_publisher(String, 'control', 1)  *# Publish String type drone control messages*  
43.     while True:  
44.         key = getKey()  *# Get key value*  
45.         if key:  
46.             print("key is :", key)  
47.             if key == '1':  *# Takeoff*  
48.                 ctr_msg.data = "takeoff"  
49.                 ctr_publisher.publish(ctr_msg)  
50.                 tello.takeoff()  
51.             elif key == '2':  *# Land*  
52.              ctr_msg.data = "land"  
53.              ctr_publisher.publish(ctr_msg)  
54.              tello.land()  
55.             elif key == 'w':  *# Move forward*  
56.                 control(("1010010001003207").encode("utf-8"))  
57.                 ctr_msg.data = "move_forward"  
58.                 ctr_publisher.publish(ctr_msg)  
59.                 tello.move_forward(20)  
60.             elif key == 's':  *# Move backward*  
61.                 control(("1010000100013207").encode("utf-8"))  
62.                 ctr_msg.data = "move_back"  
63.                 ctr_publisher.publish(ctr_msg)  
64.                 tello.move_back(20)  
65.             elif key == 'a':  *# Move left*  
66.                 ctr_msg.data = "move_left"  
67.                 ctr_publisher.publish(ctr_msg)  
68.                 tello.move_left(20)  
69.             elif key == 'd':  *# Move right*  
70.                 ctr_msg.data = "move_right"  
71.                 ctr_publisher.publish(ctr_msg)  
72.                 tello.move_right(20)  
73.             elif key == 'e':  *# Rotate clockwise*  
74.                 ctr_msg.data = "rotate_clockwise"  
75.                 ctr_publisher.publish(ctr_msg)  
76.                 tello.rotate_clockwise(10)  
77.             elif key == 'q':  *# Rotate counter-clockwise*  
78.                 ctr_msg.data = "rotate_counter_clockwise"  
79.                 ctr_publisher.publish(ctr_msg)  
80.                 tello.rotate_counter_clockwise(10)  
81.             elif key == 'r':  *# Move up*  
82.                 ctr_msg.data = "move_up"  
83.                 ctr_publisher.publish(ctr_msg)  
84.                 tello.move_up(20)  
85.             elif key == 'f':  *# Move down*  
86.                 ctr_msg.data = "move_down"  
87.                 ctr_publisher.publish(ctr_msg)  
88.                 tello.move_down(20)  
89.             elif key == 'v':  *# Stop*  
90.                 control(("0000010001003207").encode("utf-8"))  
91.                 ctr_msg.data = "stop"  
92.                 ctr_publisher.publish(ctr_msg)  
93.                 tello.land()  
94.                 time.sleep(5)  
95.                 exit(0)  

97. def main(args=None):  
98.     rclpy.init(args=args)  
99.     global node  
100.    node = Node("control0")  
101.    Thread(target=uav_camera).start()  *# Start video stream display in a separate thread to prevent interruption during drone movement*  
102.    uav_control()  
103.    rclpy.spin(node)  
104.    rclpy.shutdown()  
105.if __name__ == '__main__':  
106.    main()  

The Master image display code receives image data from the `camera0` topic and displays it via an OpenCV window. The image display code is as follows:  

1.  import rclpy  
2.  from rclpy.node import Node  
3.  from sensor_msgs.msg import CompressedImage  
4.  from cv_bridge import CvBridge  
5.  import cv2  
6.  import numpy as np  

8.  class CameraSubscriber(Node):  
9.      def __init__(self):  
10.         super().__init__('view')  
11.         self.subscriber = self.create_subscription(CompressedImage, 'camera0', self.camera_callback, 1)  *# Receive CompressedImage type compressed image messages*  
12.         self.cv_bridge = CvBridge()  
13.         print("init finish")  

15.     def camera_callback(self, msg):  
16.         print("recv from /camera")  
17.         img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)  
18.         img = cv2.resize(img, (720, 480))  
19.         cv2.imshow("Camera", img)  *# Display video stream in cv2 window*  
20.         cv2.waitKey(1)  

22. def main(args=None):  
23.     rclpy.init(args=args)  
24.     camera_subscriber = CameraSubscriber()  
25.     rclpy.spin(camera_subscriber)  
26.     rclpy.shutdown()  

28. if __name__ == '__main__':  
29.     main()  

The framework for the Master program is as follows:  

The Slave control code publishes the captured image data to the `camera1` topic for display by other programs. Upon receiving messages from the Master via ROS2 over the local network, it controls the drone's movement. The Slave control code is as follows:  

1.  import rclpy  
2.  from rclpy.node import Node  
3.  from sensor_msgs.msg import CompressedImage  
4.  from std_msgs.msg import String  
5.  from cv_bridge import CvBridge  
6.  from djitellopy import Tello  
7.  import sys, select, termios, tty  
8.  from threading import Thread  
9.  import time  

11. tello = Tello()  *# Initialize Tello*  
12. tello.connect()  *# Connect to the drone*  
13. tello.streamon()  *# Enable video stream*  
14. frame_read = tello.get_frame_read()  
15. def uav_camera():  
16.     cv_bridge = CvBridge()  
17.     img_publisher = node.create_publisher(CompressedImage, "camera1", 1)  *# Publish CompressedImage type compressed image messages*  
18.     while True:  
19.         img = frame_read.frame  
20.         img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")  
21.         img_publisher.publish(img_msg)  
22. def uav_control(msg):  
23.     print(msg.data)  
24.     if msg.data == "takeoff":  *# Takeoff*  
25.         tello.takeoff()  
26.     elif msg.data == "land":  *# Land*  
27.         tello.land()  
28.     elif msg.data == "move_forward":  *# Move forward*  
29.         tello.move_forward(20)  
30.     elif msg.data == "move_back":  *# Move backward*  
31.         tello.move_back(20)  
32.     elif msg.data == "move_left":  *# Move left*  
33.         tello.move_left(20)  
34.     elif msg.data == "move_right":  *# Move right*  
35.         tello.move_right(20)  
36.     elif msg.data == "rotate_clockwise":  *# Rotate clockwise*  
37.         tello.rotate_clockwise(10)  
38.     elif msg.data == "rotate_counter_clockwise":  *# Rotate counter-clockwise*  
39.         tello.rotate_counter_clockwise(10)  
40.     elif msg.data == "move_up":  *# Move up*  
41.         tello.move_up(20)  
42.     elif msg.data == "move_down":  *# Move down*  
43.         tello.move_down(20)  
44.     elif msg.data == "stop":  *# Stop*  
45.         tello.land()  
46.         time.sleep(5)  
47.         exit(0)  

49. def main(args=None):  
50.     rclpy.init(args=args)  
51.     global node  
52.     node = Node("control1")  
53.     Thread(target=uav_camera).start()  *# Start video stream display in a separate thread to prevent interruption during drone movement*  
54.     ctr_subscription = node.create_subscription(String, "control", uav_control, 1)  *# Receive String type drone control messages*  
55.     rclpy.spin(node)  
56.     rclpy.shutdown()  
57. if __name__ == '__main__':  
58.     main()  

The Slave image display code receives image data from the `camera1` topic and displays it via an OpenCV window. The Slave image display code is as follows:  

1.  import rclpy  
2.  from rclpy.node import Node  
3.  from sensor_msgs.msg import CompressedImage  
4.  from cv_bridge import CvBridge  
5.  import cv2  
6.  import numpy as np  

8.  class CameraSubscriber(Node):  
9.      def __init__(self):  
10.         super().__init__('view')  
11.         self.subscriber = self.create_subscription(CompressedImage, 'camera1', self.camera_callback, 1)  *# Receive CompressedImage type compressed image messages*  
12.         self.cv_bridge = CvBridge()  
13.         print("init finish")  

15.     def camera_callback(self, msg):  
16.         print("recv from /camera")  
17.         img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)  
18.         img = cv2.resize(img, (720, 480))  
19.         cv2.imshow("Camera", img)  *# Display video stream in cv2 window*  
20.         cv2.waitKey(1)  

22. def main(args=None):  
23.     rclpy.init(args=args)  
24.     camera_subscriber = CameraSubscriber()  
25.     rclpy.spin(camera_subscriber)  
26.     rclpy.shutdown()  

28. if __name__ == '__main__':  
29.     main()  

The framework for the Slave program is as follows:  

### Launching the Program  

The Master's USB port is connected to the vehicle's CAN interface via a USBCAN analyzer.  

After connecting the Master to the drone's WiFi, create two new terminals in `drone_ws` and run the following commands to control the drone and vehicle via the keyboard:  

1.  sudo -s  
2.  source /opt/tros/setup.bash  
3.  source install/setup.bash  
4.  ros2 run drone_ws control0  

1.  sudo -s  
2.  source /opt/tros/setup.bash  
3.  source install/setup.bash  
4.  ros2 run drone_ws view0  

After connecting the Slave to the drone's WiFi, create two new terminals in `drone_ws` and run the following commands to control the drone and vehicle via the keyboard:  

1.  sudo -s  
2.  source /opt/tros/setup.bash  
3.  source install/setup.bash  
4.  ros2 run drone_ws control1  

1.  sudo -s  
2.  source /opt/tros/setup.bash  
3.  source install/setup.bash  
4.  ros2 run drone_ws view1  

# Experimental Steps  

1_1. Connect to the vehicle (Bluetooth): In the Master terminal, enter `bluetoothctl`, then enter `scan on` to view the Bluetooth module address. Once the address is found, use the `pair address` command to pair, with the pairing password being `1234`. Modify the Bluetooth address in `connect_bluetooth.sh` to the one you want to pair with. In `drone_ws`, create a new terminal and run the command `./connect_bluetooth` to connect to Bluetooth.  

1_2. Connect to the vehicle (CAN): Connect the Master's USB port to the vehicle's CAN interface via a USBCAN analyzer.  

2. Launch the Master control program: On the Master, navigate to the `drone_ws` folder, open a terminal, and enter the following commands:  
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws control0  

3. Launch the Master image display program: On the Master, navigate to the `drone_ws` folder, open a terminal, and enter the following commands:  
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws view0  

4. Launch the Slave control program: On the Slave, navigate to the `drone_ws` folder, open a terminal, and enter the following commands:  
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws control1  

5. Launch the Slave image display program: On the Slave, navigate to the `drone_ws` folder, open a terminal, and enter the following commands:  
sudo -s  
source /opt/tros/setup.bash  
source install/setup.bash  
ros2 run drone_ws view1  

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

1.  import rclpy  
2.  from rclpy.node import Node  
3.  from sensor_msgs.msg import CompressedImage  
4.  from cv_bridge import CvBridge  
5.  from djitellopy import Tello  
6.  from djitellopy import TelloSwarm  
7.  import sys, select, termios, tty  
8.  from threading import Thread  
9.  import time  
10. settings = termios.tcgetattr(sys.stdin)  *# Key value initialization, read terminal attributes*  
11. *# Key value function*  
12. def getKey():  
13.     global settings  
14.     tty.setraw(sys.stdin.fileno())  
15.     rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  
16.     if rlist:  
17.         key = sys.stdin.read(1)  
18.     else:  
19.         key = ''  
20.     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  
21.     return key  

23. swarm = TelloSwarm.fromIps([  
24.     "192.168.1.101",  
25.     "192.168.1.102",  
26.     "192.168.1.103",  
27.     "192.168.1.104"  
28. ])  
29. swarm.connect()  
30. tello_takeoff = False  
31. target_pos = [[75, 75], [-75, 75], [-75, -75], [75, -75]]  *# 4 drones*  

33. def uav_control():  
34.     global tello_takeoff, target_pos  
35.     for tello in swarm:  
36.         tello.enable_mission_pads()  
37.         tello.set_mission_pad_detection_direction(0)  
38.     while True:  
39.         key = getKey()  *# Get key value*  
40.         if key:  
41.             print("key is :", key)  
42.             if key == '1':  
43.                 swarm.takeoff()  
44.                 *#time.sleep(20)*  
45.                 tello_takeoff = True  
46.             elif key == '2':  
47.                 swarm.land()  
48.             elif key == 'w':  
49.                 swarm.move_forward(30)  
50.             elif key == 's':  
51.                 swarm.move_back(30)  
52.             elif key == 'a':  
53.                 swarm.move_left(30)  
54.             elif key == 'd':  
55.                 swarm.move_right(30)  
56.             elif key == 'e':  
57.                 swarm.rotate_clockwise(30)  
58.             elif key == 'q':  
59.                 swarm.rotate_counter_clockwise(30)  
60.             elif key == 'r':  
61.                 swarm.move_up(30)  
62.             elif key == 'f':  
63.                 swarm.move_down(30)  
64.             elif key == 'v':  
65.                 swarm.disable_mission_pads()  
66.                 swarm.land()  
67.                 swarm.end()  
68.                 time.sleep(5)  
69.                 exit(0)  

71.         *# Fly around four points*  
72.         if tello_takeoff:  
73.             swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[i%4][0], target_pos[i%4][1], 80, 50, 12))  
74.             swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[(i+1)%4][0], target_pos[(i+1)%4][1], 80, 50, 12))  
75.             swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[(i+2)%4][0], target_pos[(i+2)%4][1], 80, 50, 12))  
76.             swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos[(i+3)%4][0], target_pos[(i+3)%4][1], 80, 50, 12))  

78. def main(args=None):  
79.     rclpy.init(args=args)  
80.     global node  
81.     node = Node("control")  
82.     uav_control()  
83.     rclpy.spin(node)  
84.     rclpy.shutdown()  
85. if __name__ == '__main__':  
86.     main()  

## 2.3 Launching the Program  

Run the following command to install dependencies:  

1.  pip install djitellopy  

Connect the RDK development board to the same WiFi used during the expansion board's initial setup. Switch the drone's expansion board switch to the upper position, power on the drone, and wait for the propellers to start spinning automatically.  

In the `drone_ws` workspace, run the following commands to launch the program. (For accuracy, check each drone's IP address in the router management page and modify the code accordingly.)  

1.  colcon build  
2.  source install/setup.bash  
3.  ros2 run drone_pkg control  

Once all four drones are successfully connected, press the `1` key and wait for the drones to take off. They will then automatically fly toward the target points.  