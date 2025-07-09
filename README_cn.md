# 车机协同

RDK X3 Master和Slave分别通过WiFi连接两架无人机，Master通过蓝牙或者USBCAN分析仪连接车辆下位机。Master和Slave之间使用以太网连接，当Master控制无人机0和车辆时，使用ROS2 Foxy的消息发布和订阅机制，Slave订阅Master发布的消息后控制无人机1。

## 蓝牙控制车辆

### 代码介绍

Master控制代码中将获取到的图像数据发布给camera0主题，交由其他程序显示。接收到键盘按键时通过ROS2局域网发送消息给slave以及通过蓝牙发送消息给车辆的同时控制无人机的移动。Master控制代码如下：

1.  import rclpy
2.  from rclpy.node import Node
3.  from sensor_msgs.msg import CompressedImage
4.  from std_msgs.msg import String
5.  from cv_bridge import CvBridge
6.  from djitellopy import Tello
7.  import sys, select, termios, tty
8.  from threading import Thread
9.  import time
10. import serial
11. def setto(control):
12.     global bluetooth
13.     bluetooth.flushInput()
14.     ret = bluetooth.write(str.encode(control)) _#发送蓝牙消息_
15.     time.sleep(0.01)
16. settings = termios.tcgetattr(sys.stdin) _#获取键值初始化，读取终端相关属性_
17. _#获取键值函数_
18. def getKey():
19.     global settings
20.     tty.setraw(sys.stdin.fileno())
21.     rlist, \_, _ = select.select(\[sys.stdin\], \[\], \[\], 0.1)
22.     if rlist:
23.         key = sys.stdin.read(1)
24.     else:
25.         key = ''

27.     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
28.     return key

30. tello = Tello() _#初始化Tello_
31. tello.connect() _#连接无人机_
32. tello.streamon() _#开启视频流_
33. frame_read = tello.get_frame_read()

35. def uav_camera():
36.     cv_bridge = CvBridge()
37.     img_publisher = node.create_publisher(CompressedImage, 'camera0', 1) _#发布CompressedImage类型压缩图片消息_
38.     while True:
39.         img = frame_read.frame _#获取一帧视频_
40.         img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
41.         img_publisher.publish(img_msg)
42. def uav_control():
43.     ctr_msg = String()
44.     ctr_publisher = node.create_publisher(String, 'control', 1) _#发布String类型无人机控制消息给Slave_
45.     while True:
46.         key = getKey() _#获取键值_
47.         if key:
48.             print("key is :", key)
49.             if key == '1': _#起飞_
50.                 ctr_msg.data = "takeoff"
51.                 ctr_publisher.publish(ctr_msg)
52.                 tello.takeoff()
53.             elif key == '2': _#降落_
54.              ctr_msg.data = "land"
55.              ctr_publisher.publish(ctr_msg)
56.              tello.land()
57.             elif key == 'w': _#前进_
58.                 setto("&lt;+00+10&gt;")
59.                 ctr_msg.data = "move_forward"
60.                 ctr_publisher.publish(ctr_msg)
61.                 tello.move_forward(20)
62.             elif key == 's': _#后退_
63.                 setto("&lt;+00-10&gt;")
64.                 ctr_msg.data = "move_back"
65.                 ctr_publisher.publish(ctr_msg)
66.                 tello.move_back(20)
67.             elif key == 'a': _#左移_
68.                 ctr_msg.data = "move_left"
69.                 ctr_publisher.publish(ctr_msg)
70.                 tello.move_left(20)
71.             elif key == 'd': _#右移_
72.                 ctr_msg.data = "move_right"
73.                 ctr_publisher.publish(ctr_msg)
74.                 tello.move_right(20)
75.             elif key == 'e': _#顺时针转向_
76.                 ctr_msg.data = "rotate_clockwise"
77.                 ctr_publisher.publish(ctr_msg)
78.                 tello.rotate_clockwise(10)
79.             elif key == 'q': _#逆时针转向_
80.                 ctr_msg.data = "rotate_counter_clockwise"
81.                 ctr_publisher.publish(ctr_msg)
82.                 tello.rotate_counter_clockwise(10)
83.             elif key == 'r': _#升高_
84.                 ctr_msg.data = "move_up"
85.                 ctr_publisher.publish(ctr_msg)
86.                 tello.move_up(20)
87.             elif key == 'f': _#降低_
88.                 ctr_msg.data = "move_down"
89.                 ctr_publisher.publish(ctr_msg)
90.                 tello.move_down(20)
91.             elif key == 'v': _#停止_
92.                 setto("&lt;+00+00&gt;")
93.                 ctr_msg.data = "stop"
94.                 ctr_publisher.publish(ctr_msg)
95.                 tello.land()
96.                 time.sleep(5)
97.                 exit(0)

99. def main(args=None):
100.    global bluetooth
101.    bluetooth = serial.Serial("/dev/rfcomm1",115200) _#建立蓝牙连接_
102.    rclpy.init(args=args)
103.    global node
104.    node = Node("control0")
105.    Thread(target=uav_camera).start() _#在多线程中开启视频流显示程序，防止无人机运动过程中视频显示中断_
106.    uav_control()
107.    rclpy.spin(node)
108.    rclpy.shutdown()
109.    bluetooth.close() _#关闭蓝牙连接_
110.if \__name__ == '\__main_\_':
111.    main()

Master图像显示代码中接收到camera0主题的图像数据并通过opencv窗口显示。Master图像显示代码如下：

1.  import rclpy
2.  from rclpy.node import Node
3.  from sensor_msgs.msg import CompressedImage
4.  from cv_bridge import CvBridge
5.  import cv2
6.  import numpy as np

8.  class CameraSubscriber(Node):
9.      def \__init_\_(self):
10.         super().\__init_\_('view')
11.         self.subscriber = self.create_subscription(CompressedImage, 'camera0', self.camera_callback, 1) _#接收CompressedImage类型压缩图片消息_
12.         self.cv_bridge = CvBridge()
13.         print("init finish")

15.     def camera_callback(self, msg):
16.         print("recv from /camera")
17.         img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
18.         img = cv2.resize(img, (720, 480))
19.         cv2.imshow("Camera", img) _#cv2窗口显示视频流_
20.         cv2.waitKey(1)

22. def main(args=None):
23.     rclpy.init(args=args)
24.     camera_subscriber = CameraSubscriber()
25.     rclpy.spin(camera_subscriber)
26.     rclpy.shutdown()

28. if \__name__ == '\__main_\_':
29.     main()

Master程序对应框架部分如下：

Slave控制代码中将获取到的图像数据发布给camera1主题，交由其他程序显示。通过ROS2局域网接收到master发送的消息后控制无人机的移动。Slave控制代码如下：

1.  import rclpy
2.  from rclpy.node import Node
3.  from sensor_msgs.msg import CompressedImage
4.  from std_msgs.msg import String
5.  from cv_bridge import CvBridge
6.  from djitellopy import Tello
7.  import sys, select, termios, tty
8.  from threading import Thread
9.  import time

11. tello = Tello() _#初始化Tello_
12. tello.connect() _#连接无人机_
13. tello.streamon() _#启动视频流_
14. frame_read = tello.get_frame_read()
15. def uav_camera():
16.     cv_bridge = CvBridge()
17.     img_publisher = node.create_publisher(CompressedImage, "camera1", 1) _#发送Compressed类型压缩图片消息_
18.     while True:
19.         img = frame_read.frame
20.         img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
21.         img_publisher.publish(img_msg)
22. def uav_control(msg):
23.     print(msg.data)
24.     if msg.data == "takeoff": _#起飞_
25.         tello.takeoff()
26.     elif msg.data == "land": _#降落_
27.         tello.land()
28.     elif msg.data == "move_forward": _#前进_
29.         tello.move_forward(20)
30.     elif msg.data == "move_back": _#后退_
31.         tello.move_back(20)
32.     elif msg.data == "move_left": _#左移_
33.         tello.move_left(20)
34.     elif msg.data == "move_right": _#右移_
35.         tello.move_right(20)
36.     elif msg.data == "rotate_clockwise": _#顺时针旋转_
37.         tello.rotate_clockwise(10)
38.     elif msg.data == "rotate_counter_clockwise": _#逆时针旋转_
39.         tello.rotate_counter_clockwise(10)
40.     elif msg.data == "move_up": _#升高_
41.         tello.move_up(20)
42.     elif msg.data == "move_down": _#降低_
43.         tello.move_down(20)
44.     elif msg.data == "stop": _#停止_
45.         tello.land()
46.         time.sleep(5)
47.         exit(0)

49. def main(args=None):
50.     rclpy.init(args=args)
51.     global node
52.     node = Node("control1")
53.     Thread(target=uav_camera).start() _#在多线程中开启视频流显示程序，防止无人机运动过程中视频显示中断_
54.     ctr_subscription = node.create_subscription(String, "control", uav_control, 1) _#接收Master发送的String类型无人机控制消息_
55.     rclpy.spin(node)
56.     rclpy.shutdown()
57. if \__name__ == '\__main_\_':
58.     main()

Slave图像显示代码中接收到camera1主题的图像数据并通过opencv窗口显示。Slave图像显示代码如下：

1.  import rclpy
2.  from rclpy.node import Node
3.  from sensor_msgs.msg import CompressedImage
4.  from cv_bridge import CvBridge
5.  import cv2
6.  import numpy as np

8.  class CameraSubscriber(Node):
9.      def \__init_\_(self):
10.         super().\__init_\_('view')
11.         self.subscriber = self.create_subscription(CompressedImage, 'camera1', self.camera_callback, 1) _#接收CompressedImage类型的压缩图片消息_
12.         self.cv_bridge = CvBridge()
13.         print("init finish")

15.     def camera_callback(self, msg):
16.         print("recv from /camera")
17.         img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
18.         img = cv2.resize(img, (720, 480))
19.         cv2.imshow("Camera", img) _#cv2窗口显示视频流_
20.         cv2.waitKey(1)

22. def main(args=None):
23.     rclpy.init(args=args)
24.     camera_subscriber = CameraSubscriber()
25.     rclpy.spin(camera_subscriber)
26.     rclpy.shutdown()

28. if \__name__ == '\__main_\_':
29.     main()

Slave程序对应框架部分如下：

### 启动程序

蓝牙地址需要在Master终端中输入bluetoothctl后输入scan on查看，当搜索到蓝牙模块地址后使用pair 地址命令进行配对，配对密码为1234。将connect_bluetooth.sh中蓝牙地址修改为需要配对的蓝牙地址。

在drone_ws内新建输入命令连接蓝牙：

1.  ./connect_bluetooth.sh

Master连接无人机wifi后在drone_ws内新建两个终端运行分别以下命令，在终端内用键盘控制无人机和车辆：

1.  sudo -s
2.  source /opt/tros/setup.bash
3.  source install/setup.bash
4.  ros2 run drone_ws control0

1.  sudo -s
2.  source /opt/tros/setup.bash
3.  source install/setup.bash
4.  ros2 run drone_ws view0

Slave连接无人机wifi后在drone_ws内新建两个终端运行分别以下命令，在终端内用键盘控制无人机和车辆：

1.  sudo -s
2.  source /opt/tros/setup.bash
3.  source install/setup.bash
4.  ros2 run drone_ws control1

1.  sudo -s
2.  source /opt/tros/setup.bash
3.  source install/setup.bash
4.  ros2 run drone_ws view1

## CAN控制车辆

### 代码介绍

Master控制代码中将获取到的图像数据发布给camera主题，交由其他程序显示。接收到键盘按键时通过ROS2局域网发送消息给slave，以及调用drone_ws下canusb.c编译成的canusb.so文件中的control函数，将发送CAN消息给车辆的同时控制无人机的移动。Master控制代码如下：

1.  import rclpy
2.  from rclpy.node import Node
3.  from sensor_msgs.msg import CompressedImage
4.  from std_msgs.msg import String
5.  from cv_bridge import CvBridge
6.  from djitellopy import Tello
7.  import sys, select, termios, tty
8.  from threading import Thread
9.  import time
10. from ctypes import \*
11. mylib = CDLL("/home/sunrise/drone_ws/canusb.so") _#加载动态链接库_
12. control = mylib.control _#调用动态链接库中函数_
13. received_payload = b''
14. settings = termios.tcgetattr(sys.stdin) _#获取键值初始化，读取终端相关属性_
15. _#获取键值函数_
16. def getKey():
17.     global settings
18.     tty.setraw(sys.stdin.fileno())
19.     rlist, \_, _ = select.select(\[sys.stdin\], \[\], \[\], 0.1)
20.     if rlist:
21.         key = sys.stdin.read(1)
22.     else:
23.         key = ''

25.     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
26.     return key

28. tello = Tello() _#初始化Tello_
29. tello.connect() _#连接无人机_
30. tello.streamon() _#开启视频流_
31. frame_read = tello.get_frame_read()

33. def uav_camera():
34.     cv_bridge = CvBridge()
35.     img_publisher = node.create_publisher(CompressedImage, 'camera0', 1) _#发布CompressedImage类型的压缩图片消息_
36.     while True:
37.         img = frame_read.frame
38.         img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
39.         img_publisher.publish(img_msg)
40. def uav_control():
41.     ctr_msg = String()
42.     ctr_publisher = node.create_publisher(String, 'control', 1) _#发布String类型的无人机控制消息_
43.     while True:
44.         key = getKey() _#获取键值_
45.         if key:
46.             print("key is :", key)
47.             if key == '1': _#起飞_
48.                 ctr_msg.data = "takeoff"
49.                 ctr_publisher.publish(ctr_msg)
50.                 tello.takeoff()
51.             elif key == '2': _#降落_
52.              ctr_msg.data = "land"
53.              ctr_publisher.publish(ctr_msg)
54.              tello.land()
55.             elif key == 'w': _#前进_
56.                 control(("1010010001003207").encode("utf-8"))
57.                 ctr_msg.data = "move_forward"
58.                 ctr_publisher.publish(ctr_msg)
59.                 tello.move_forward(20)
60.             elif key == 's': _#后退_
61.                 control(("1010000100013207").encode("utf-8"))
62.                 ctr_msg.data = "move_back"
63.                 ctr_publisher.publish(ctr_msg)
64.                 tello.move_back(20)
65.             elif key == 'a': _#左移_
66.                 ctr_msg.data = "move_left"
67.                 ctr_publisher.publish(ctr_msg)
68.                 tello.move_left(20)
69.             elif key == 'd': _#右移_
70.                 ctr_msg.data = "move_right"
71.                 ctr_publisher.publish(ctr_msg)
72.                 tello.move_right(20)
73.             elif key == 'e': _#顺时针旋转_
74.                 ctr_msg.data = "rotate_clockwise"
75.                 ctr_publisher.publish(ctr_msg)
76.                 tello.rotate_clockwise(10)
77.             elif key == 'q': _#逆时针旋转_
78.                 ctr_msg.data = "rotate_counter_clockwise"
79.                 ctr_publisher.publish(ctr_msg)
80.                 tello.rotate_counter_clockwise(10)
81.             elif key == 'r': _#升高_
82.                 ctr_msg.data = "move_up"
83.                 ctr_publisher.publish(ctr_msg)
84.                 tello.move_up(20)
85.             elif key == 'f': _#降低_
86.                 ctr_msg.data = "move_down"
87.                 ctr_publisher.publish(ctr_msg)
88.                 tello.move_down(20)
89.             elif key == 'v': _#停止_
90.                 control(("0000010001003207").encode("utf-8"))
91.                 ctr_msg.data = "stop"
92.                 ctr_publisher.publish(ctr_msg)
93.                 tello.land()
94.                 time.sleep(5)
95.                 exit(0)

97. def main(args=None):
98.     rclpy.init(args=args)
99.     global node
100.    node = Node("control0")
101.    Thread(target=uav_camera).start() _#在多线程中开启视频流显示程序，防止无人机运动过程中视频显示中断_
102.    uav_control()
103.    rclpy.spin(node)
104.    rclpy.shutdown()
105.if \__name__ == '\__main_\_':
106.    main()

Master图像显示代码中接收到camera0主题的图像数据并通过opencv窗口显示。图像显示代码如下：

1.  import rclpy
2.  from rclpy.node import Node
3.  from sensor_msgs.msg import CompressedImage
4.  from cv_bridge import CvBridge
5.  import cv2
6.  import numpy as np

8.  class CameraSubscriber(Node):
9.      def \__init_\_(self):
10.         super().\__init_\_('view')
11.         self.subscriber = self.create_subscription(CompressedImage, 'camera0', self.camera_callback, 1) _#接收CompressedImage类型的压缩图片消息_
12.         self.cv_bridge = CvBridge()
13.         print("init finish")

15.     def camera_callback(self, msg):
16.         print("recv from /camera")
17.         img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
18.         img = cv2.resize(img, (720, 480))
19.         cv2.imshow("Camera", img) _#cv2窗口显示视频流_
20.         cv2.waitKey(1)

22. def main(args=None):
23.     rclpy.init(args=args)
24.     camera_subscriber = CameraSubscriber()
25.     rclpy.spin(camera_subscriber)
26.     rclpy.shutdown()

28. if \__name__ == '\__main_\_':
29.     main()

Master程序对应框架部分如下：

Slave控制代码中将获取到的图像数据发布给camera1主题，交由其他程序显示。通过ROS2局域网接收到master发送的消息后控制无人机的移动。Slave控制代码如下：

1.  import rclpy
2.  from rclpy.node import Node
3.  from sensor_msgs.msg import CompressedImage
4.  from std_msgs.msg import String
5.  from cv_bridge import CvBridge
6.  from djitellopy import Tello
7.  import sys, select, termios, tty
8.  from threading import Thread
9.  import time

11. tello = Tello() _#初始化Tello_
12. tello.connect() _#连接无人机_
13. tello.streamon() _#开启视频流_
14. frame_read = tello.get_frame_read()
15. def uav_camera():
16.     cv_bridge = CvBridge()
17.     img_publisher = node.create_publisher(CompressedImage, "camera1", 1) _#发布Compressed类型的压缩图片消息_
18.     while True:
19.         img = frame_read.frame
20.         img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
21.         img_publisher.publish(img_msg)
22. def uav_control(msg):
23.     print(msg.data)
24.     if msg.data == "takeoff": _#起飞_
25.         tello.takeoff()
26.     elif msg.data == "land": _#降落_
27.         tello.land()
28.     elif msg.data == "move_forward": _#前进_
29.         tello.move_forward(20)
30.     elif msg.data == "move_back": _#后退_
31.         tello.move_back(20)
32.     elif msg.data == "move_left": _#左移_
33.         tello.move_left(20)
34.     elif msg.data == "move_right": _#右移_
35.         tello.move_right(20)
36.     elif msg.data == "rotate_clockwise": _#顺时针旋转_
37.         tello.rotate_clockwise(10)
38.     elif msg.data == "rotate_counter_clockwise": _#逆时针旋转_
39.         tello.rotate_counter_clockwise(10)
40.     elif msg.data == "move_up": _#升高_
41.         tello.move_up(20)
42.     elif msg.data == "move_down": _#降低_
43.         tello.move_down(20)
44.     elif msg.data == "stop": _#停止_
45.         tello.land()
46.         time.sleep(5)
47.         exit(0)

49. def main(args=None):
50.     rclpy.init(args=args)
51.     global node
52.     node = Node("control1")
53.     Thread(target=uav_camera).start() _#在多线程中开启视频流显示程序，防止无人机运动过程中视频显示中断_
54.     ctr_subscription = node.create_subscription(String, "control", uav_control, 1) _#接收String类型的无人机控制消息_
55.     rclpy.spin(node)
56.     rclpy.shutdown()
57. if \__name__ == '\__main_\_':
58.     main()

Slave图像显示代码中接收到camera1主题的图像数据并通过opencv窗口显示。Slave图像显示代码如下：

1.  import rclpy
2.  from rclpy.node import Node
3.  from sensor_msgs.msg import CompressedImage
4.  from cv_bridge import CvBridge
5.  import cv2
6.  import numpy as np

8.  class CameraSubscriber(Node):
9.      def \__init_\_(self):
10.         super().\__init_\_('view')
11.         self.subscriber = self.create_subscription(CompressedImage, 'camera1', self.camera_callback, 1) _#接收CompressedImage类型的压缩图片消息_
12.         self.cv_bridge = CvBridge()
13.         print("init finish")

15.     def camera_callback(self, msg):
16.         print("recv from /camera")
17.         img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
18.         img = cv2.resize(img, (720, 480))
19.         cv2.imshow("Camera", img) _#cv2窗口显示视频流_
20.         cv2.waitKey(1)

22. def main(args=None):
23.     rclpy.init(args=args)
24.     camera_subscriber = CameraSubscriber()
25.     rclpy.spin(camera_subscriber)
26.     rclpy.shutdown()

28. if \__name__ == '\__main_\_':
29.     main()

Slave程序对应框架部分如下：

### 启动程序

Master的USB口通过USBCAN分析仪连接车辆CAN接口。

Master连接无人机wifi后在drone_ws内新建两个终端运行分别以下命令，在终端内用键盘控制无人机和车辆：

1.  sudo -s
2.  source /opt/tros/setup.bash
3.  source install/setup.bash
4.  ros2 run drone_ws control0

1.  sudo -s
2.  source /opt/tros/setup.bash
3.  source install/setup.bash
4.  ros2 run drone_ws view0

Slave连接无人机wifi后在drone_ws内新建两个终端运行分别以下命令，在终端内用键盘控制无人机和车辆：

1.  sudo -s
2.  source /opt/tros/setup.bash
3.  source install/setup.bash
4.  ros2 run drone_ws control1

1.  sudo -s
2.  source /opt/tros/setup.bash
3.  source install/setup.bash
4.  ros2 run drone_ws view1

# 实验步骤

1_1.连接车辆（蓝牙）：在Master终端中输入bluetoothctl后输入scan on查看，当搜索到蓝牙模块地址后使用pair 地址命令进行配对，配对密码为1234。将connect_bluetooth.sh中蓝牙地址修改为需要配对的蓝牙地址。在drone_ws内新建输入命令./connect_bluetooth连接蓝牙。

1_2.连接车辆（CAN）：Master的USB口通过USBCAN分析仪连接车辆CAN接口。

2.启动Master控制程序：在Master上进入drone_ws文件夹，打开终端输入以下命令：
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws control0

3.启动Master图像显示程序：在Master上进入drone_ws文件夹，打开终端输入以下命令：
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws view0

4.启动Slave控制程序：在Slave上进入drone_ws文件夹，打开终端输入以下命令：
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws control1

5.启动Slave图像显示程序：在Slave上进入drone_ws文件夹，打开终端输入以下命令：
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws view1
6.控制无人机和车：在Master控制程序运行的终端中使用按键控制两架无人机和车。

# 多机编队

RDK开发板和无人机都连接上同一无线路由器的WiFi，无人机连接成功后机翼自动开始旋转，RDK开发板连接成功后可以在路由器管理页面查看各设备的IP地址。RDK开发板通过WiFi向无人机编队发送命令进行控制。

## 扩展板恢复初始设置

扩展板microUSB另一端连接电脑。

进入https://mindplus.cc/，下载安装Mind+。

打开Mind+，选择上传模式；随后选择连接设备，选择左下角扩展，随后选择RoboMaster TT(ESP32)后返回。

此时选择设备栏里的恢复设备初始设置，直到右下角显示该数据；如果电脑上没有安装串口驱动，可以选择一键安装。

将扩展板usb接在无人机上，接通无人机电源，并将扩展板上的开关拨到下方，即单机模式。

在Mind+中选择实时模式，并点击左下角扩展，选择功能模块下的RoboMaster TT（单机）。

然后点击返回，依次点击功能模块和感叹号。

在电脑上通过wifi连接无人机网络，随后连接该无人机；如果连接不上则刷新后多尝试几次，随后连接成功。

将切换为station模式的模块拖入中间空白处，并且修改为准备好的wifi路由器热点名称和密码。

随后电脑连入同一个wifi，并双击该修改好的模块，可以看到OK,drone will reboot in 3s的字样。

将无人机扩展板开关拨到上方，等待一段时间，当无人机自动连接wifi后螺旋桨将会开始转动。

在路由器管理页面找到已连接的无人机，可以找到每个无人机对应的IP地址。

## 2.2 代码介绍

通过djitellopy库的TelloSwarm连接4架无人机（指定IP地址，为保证准确可在路由器管理网址中查看），启用任务板定位功能，设定下方摄像头检测任务板。

首先将四架无人机按顺序放在飞行地图指定位置，随后通过键盘按键1触发集群起飞，起飞后，四架无人机按预设坐标target_pos（四个75cm边长的矩形顶点）循环飞行，每架无人机依次飞向下一个目标点（如Drone1: A→B→C→D→A），同时使用go_xyz_speed_mid实现高精度定位飞行（依赖飞行地图的坐标反馈）。

新建ROS2工作空间drone_ws和功能包drone_pkg，并在drone_pkg下新建程序control.py，内容如下：

1.  import rclpy
2.  from rclpy.node import Node
3.  from sensor_msgs.msg import CompressedImage
4.  from cv_bridge import CvBridge
5.  from djitellopy import Tello
6.  from djitellopy import TelloSwarm
7.  import sys, select, termios, tty
8.  from threading import Thread
9.  import time
10. settings = termios.tcgetattr(sys.stdin) _#获取键值初始化，读取终端相关属性_
11. _#获取键值函数_
12. def getKey():
13.     global settings
14.     tty.setraw(sys.stdin.fileno())
15.     rlist, \_, _ = select.select(\[sys.stdin\], \[\], \[\], 0.1)
16.     if rlist:
17.         key = sys.stdin.read(1)
18.     else:
19.         key = ''
20.     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
21.     return key

23. swarm = TelloSwarm.fromIps(\[
24.     "192.168.1.101",
25.     "192.168.1.102",
26.     "192.168.1.103",
27.     "192.168.1.104"
28. \])
29. swarm.connect()
30. tello_takeoff = False
31. target_pos = \[\[75, 75\], \[-75, 75\], \[-75, -75\], \[75, -75\]\] _#4 drones_

33. def uav_control():
34.     global tello_takeoff, target_pos
35.     for tello in swarm:
36.         tello.enable_mission_pads()
37.         tello.set_mission_pad_detection_direction(0)
38.     while True:
39.         key = getKey() _#获取键值_
40.         if key:
41.             print("key is :", key)
42.             if key == '1':
43.                 swarm.takeoff()
44.                 _#time.sleep(20)_
45.                 tello_takeoff = True
46.             elif key == '2':
47.                 swarm.land()
48.             elif key == 'w':
49.                 swarm.move_forward(30)
50.             elif key == 's':
51.                 swarm.move_back(30)
52.             elif key == 'a':
53.                 swarm.move_left(30)
54.             elif key == 'd':
55.                 swarm.move_right(30)
56.             elif key == 'e':
57.                 swarm.rotate_clockwise(30)
58.             elif key == 'q':
59.                 swarm.rotate_counter_clockwise(30)
60.             elif key == 'r':
61.                 swarm.move_up(30)
62.             elif key == 'f':
63.                 swarm.move_down(30)
64.             elif key == 'v':
65.                 swarm.disable_mission_pads()
66.                 swarm.land()
67.                 swarm.end()
68.                 time.sleep(5)
69.                 exit(0)

71.         _#围绕四个点飞行_
72.         if tello_takeoff:
73.             swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos\[i%4\]\[0\], target_pos\[i%4\]\[1\], 80, 50, 12))
74.             swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos\[(i+1)%4\]\[0\], target_pos\[(i+1)%4\]\[1\], 80, 50, 12))
75.             swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos\[(i+2)%4\]\[0\], target_pos\[(i+2)%4\]\[1\], 80, 50, 12))
76.             swarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(target_pos\[(i+3)%4\]\[0\], target_pos\[(i+3)%4\]\[1\], 80, 50, 12))

78. def main(args=None):
79.     rclpy.init(args=args)
80.     global node
81.     node = Node("control")
82.     uav_control()
83.     rclpy.spin(node)
84.     rclpy.shutdown()
85. if \__name__ == '\__main_\_':
86.     main()

## 2.3 启动程序

输入以下命令安装依赖：

1.  pip install djitellopy

将RDK开发板连接到无人机扩展板初始设置时设置的同一WiFi下，将无人机扩展板开关拨到上方，打开无人机电源等待机翼自动旋转。

在工作空间drone_ws中输入以下命令启动程序。（为保证准确可在路由器管理网址中查看各无人机IP并在代码中修改）

1.  colcon build
2.  source install/setup.bash
3.  ros2 run drone_pkg control

当四架无人机都连接成功后按下按键1等待无人机起飞，随后无人机自动朝着目标前进。