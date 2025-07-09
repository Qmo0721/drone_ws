# 车机协同

RDK X3 Master和Slave分别通过WiFi连接两架无人机,Master通过蓝牙或者USBCAN分析仪连接车辆下位机。Master和Slave之间使用以太网连接,当Master控制无人机0和车辆时,使用ROS2 Foxy的消息发布和订阅机制,Slave订阅Master发布的消息后控制无人机1。

## 蓝牙控制车辆

### 代码介绍

Master控制代码中将获取到的图像数据发布给camera0主题,交由其他程序显示。接收到键盘按键时通过ROS2局域网发送消息给slave以及通过蓝牙发送消息给车辆的同时控制无人机的移动。Master控制代码如下:

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
    ret = bluetooth.write(str.encode(control)) #发送蓝牙消息
    time.sleep(0.01)
settings = termios.tcgetattr(sys.stdin) #获取键值初始化,读取终端相关属性
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

tello = Tello() #初始化Tello
tello.connect() #连接无人机
tello.streamon() #开启视频流
frame_read = tello.get_frame_read()

def uav_camera():
    cv_bridge = CvBridge()
    img_publisher = node.create_publisher(CompressedImage, 'camera0', 1) #发布CompressedImage类型压缩图片消息
    while True:
        img = frame_read.frame #获取一帧视频
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
        img_publisher.publish(img_msg)
def uav_control():
    ctr_msg = String()
    ctr_publisher = node.create_publisher(String, 'control', 1) #发布String类型无人机控制消息给Slave
    while True:
        key = getKey() #获取键值
        if key:
            print("key is :", key)
            if key == '1': #起飞
                ...

def main(args=None):
    global bluetooth
    bluetooth = serial.Serial("/dev/rfcomm1",115200) #建立蓝牙连接
    rclpy.init(args=args)
    global node
    node = Node("control0")
    Thread(target=uav_camera).start() #在多线程中开启视频流显示程序,防止无人机运动过程中视频显示中断
    uav_control()
    rclpy.spin(node)
    rclpy.shutdown()
    bluetooth.close() #关闭蓝牙连接
if __name__ == '__main__':
    main()
```

Master图像显示代码中接收到camera0主题的图像数据并通过opencv窗口显示。Master图像显示代码如下:

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
        self.subscriber = self.create_subscription(CompressedImage, 'camera0', self.camera_callback, 1) #接收CompressedImage类型压缩图片消息
        self.cv_bridge = CvBridge()
        print("init finish")

    def camera_callback(self, msg):
        print("recv from /camera")
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.resize(img, (720, 480))
        cv2.imshow("Camera", img) #cv2窗口显示视频流
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Master程序对应框架部分如下:

Slave控制代码中将获取到的图像数据发布给camera1主题,交由其他程序显示。通过ROS2局域网接收到master发送的消息后控制无人机的移动。Slave控制代码如下:

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

tello = Tello() #初始化Tello
tello.connect() #连接无人机
tello.streamon() #启动视频流
frame_read = tello.get_frame_read()
def uav_camera():
    cv_bridge = CvBridge()
    img_publisher = node.create_publisher(CompressedImage, "camera1", 1) #发送Compressed类型压缩图片消息
    while True:
        img = frame_read.frame
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
        img_publisher.publish(img_msg)
def uav_control(msg):
    print(msg.data)
    if msg.data == "takeoff": #起飞
        tello.takeoff()
    elif msg.data == "land": #降落
        tello.land()
    ...

def main(args=None):
    rclpy.init(args=args)
    global node
    node = Node("control1")
    Thread(target=uav_camera).start() #在多线程中开启视频流显示程序,防止无人机运动过程中视频显示中断
    ctr_subscription = node.create_subscription(String, "control", uav_control, 1) #接收Master发送的String类型无人机控制消息
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
```

Slave图像显示代码中接收到camera1主题的图像数据并通过opencv窗口显示。Slave图像显示代码如下:

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
        self.subscriber = self.create_subscription(CompressedImage, 'camera1', self.camera_callback, 1) #接收CompressedImage类型的压缩图片消息
        self.cv_bridge = CvBridge()
        print("init finish")

    def camera_callback(self, msg):
        print("recv from /camera")
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.resize(img, (720, 480))
        cv2.imshow("Camera", img) #cv2窗口显示视频流
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Slave程序对应框架部分如下:

### 启动程序

蓝牙地址需要在Master终端中输入bluetoothctl后输入scan on查看,当搜索到蓝牙模块地址后使用pair 地址命令进行配对,配对密码为1234。将connect_bluetooth.sh中蓝牙地址修改为需要配对的蓝牙地址。

在drone_ws内新建输入命令连接蓝牙:

```bash
./connect_bluetooth.sh
```

Master连接无人机wifi后在drone_ws内新建两个终端运行分别以下命令,在终端内用键盘控制无人机和车辆:

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

Slave连接无人机wifi后在drone_ws内新建两个终端运行分别以下命令,在终端内用键盘控制无人机和车辆:

```bash
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws control1
```

```bash
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws view1
```

## CAN控制车辆

### 代码介绍

Master控制代码中将获取到的图像数据发布给camera主题,交由其他程序显示。接收到键盘按键时通过ROS2局域网发送消息给slave,以及调用drone_ws下canusb.c编译成的canusb.so文件中的control函数,将发送CAN消息给车辆的同时控制无人机的移动。Master控制代码如下:

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
mylib = CDLL("/home/sunrise/drone_ws/canusb.so") #加载动态链接库
control = mylib.control #调用动态链接库中函数
received_payload = b''
settings = termios.tcgetattr(sys.stdin) #获取键值初始化,读取终端相关属性
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

tello = Tello() #初始化Tello
tello.connect() #连接无人机
tello.streamon() #开启视频流
frame_read = tello.get_frame_read()

def uav_camera():
    cv_bridge = CvBridge()
    img_publisher = node.create_publisher(CompressedImage, 'camera0', 1) #发布CompressedImage类型的压缩图片消息
    while True:
        img = frame_read.frame
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
        img_publisher.publish(img_msg)
def uav_control():
    ctr_msg = String()
    ctr_publisher = node.create_publisher(String, 'control', 1) #发布String类型的无人机控制消息
    while True:
        key = getKey() #获取键值
        if key:
            print("key is :", key)
            if key == '1': #起飞
                ...

def main(args=None):
    rclpy.init(args=args)
    global node
    node = Node("control0")
    Thread(target=uav_camera).start() #在多线程中开启视频流显示程序,防止无人机运动过程中视频显示中断
    uav_control()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
```

Master图像显示代码中接收到camera0主题的图像数据并通过opencv窗口显示。图像显示代码如下:

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
        self.subscriber = self.create_subscription(CompressedImage, 'camera0', self.camera_callback, 1) #接收CompressedImage类型的压缩图片消息
        self.cv_bridge = CvBridge()
        print("init finish")

    def camera_callback(self, msg):
        print("recv from /camera")
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.resize(img, (720, 480))
        cv2.imshow("Camera", img) #cv2窗口显示视频流
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Master程序对应框架部分如下:

Slave控制代码中将获取到的图像数据发布给camera1主题,交由其他程序显示。通过ROS2局域网接收到master发送的消息后控制无人机的移动。Slave控制代码如下:

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

tello = Tello() #初始化Tello
tello.connect() #连接无人机
tello.streamon() #开启视频流
frame_read = tello.get_frame_read()
def uav_camera():
    cv_bridge = CvBridge()
    img_publisher = node.create_publisher(CompressedImage, "camera1", 1) #发布Compressed类型的压缩图片消息
    while True:
        img = frame_read.frame
        img_msg = cv_bridge.cv2_to_compressed_imgmsg(img, "jpeg")
        img_publisher.publish(img_msg)
def uav_control(msg):
    print(msg.data)
    if msg.data == "takeoff": #起飞
        tello.takeoff()
    elif msg.data == "land": #降落
        tello.land()
    ...

def main(args=None):
    rclpy.init(args=args)
    global node
    node = Node("control1")
    Thread(target=uav_camera).start() #在多线程中开启视频流显示程序,防止无人机运动过程中视频显示中断
    ctr_subscription = node.create_subscription(String, "control", uav_control, 1) #接收String类型的无人机控制消息
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
```

Slave图像显示代码中接收到camera1主题的图像数据并通过opencv窗口显示。Slave图像显示代码如下:

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
        self.subscriber = self.create_subscription(CompressedImage, 'camera1', self.camera_callback, 1) #接收CompressedImage类型的压缩图片消息
        self.cv_bridge = CvBridge()
        print("init finish")

    def camera_callback(self, msg):
        print("recv from /camera")
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.resize(img, (720, 480))
        cv2.imshow("Camera", img) #cv2窗口显示视频流
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Slave程序对应框架部分如下:

### 启动程序

Master的USB口通过USBCAN分析仪连接车辆CAN接口。

Master连接无人机wifi后在drone_ws内新建两个终端运行分别以下命令,在终端内用键盘控制无人机和车辆:

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

Slave连接无人机wifi后在drone_ws内新建两个终端运行分别以下命令,在终端内用键盘控制无人机和车辆:

```bash
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws control1
```

```bash
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws view1
```

# 实验步骤

1_1.连接车辆(蓝牙):在Master终端中输入bluetoothctl后输入scan on查看,当搜索到蓝牙模块地址后使用pair 地址命令进行配对,配对密码为1234。将connect_bluetooth.sh中蓝牙地址修改为需要配对的蓝牙地址。在drone_ws内新建输入命令./connect_bluetooth连接蓝牙。

1_2.连接车辆(CAN):Master的USB口通过USBCAN分析仪连接车辆CAN接口。

2.启动Master控制程序:在Master上进入drone_ws文件夹,打开终端输入以下命令:
```bash
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws control0
```

3.启动Master图像显示程序:在Master上进入drone_ws文件夹,打开终端输入以下命令:
```bash
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws view0
```

4.启动Slave控制程序:在Slave上进入drone_ws文件夹,打开终端输入以下命令:
```bash
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws control1
```

5.启动Slave图像显示程序:在Slave上进入drone_ws文件夹,打开终端输入以下命令:
```bash
sudo -s
source /opt/tros/setup.bash
source install/setup.bash
ros2 run drone_ws view1
```
6.控制无人机和车:在Master控制程序运行的终端中使用按键控制两架无人机和车。

# 多机编队

RDK开发板和无人机都连接上同一无线路由器的WiFi,无人机连接成功后机翼自动开始旋转,RDK开发板连接成功后可以在路由器管理页面查看各设备的IP地址。RDK开发板通过WiFi向无人机编队发送命令进行控制。

## 扩展板恢复初始设置

扩展板microUSB另一端连接电脑。

进入https://mindplus.cc/,下载安装Mind+。

打开Mind+,选择上传模式;随后选择连接设备,选择左下角扩展,随后选择RoboMaster TT(ESP32)后返回。

此时选择设备栏里的恢复设备初始设置,直到右下角显示该数据;如果电脑上没有安装串口驱动,可以选择一键安装。

将扩展板usb接在无人机上,接通无人机电源,并将扩展板上的开关拨到下方,即单机模式。

在Mind+中选择实时模式,并点击左下角扩展,选择功能模块下的RoboMaster TT(单机)。

然后点击返回,依次点击功能模块和感叹号。

在电脑上通过wifi连接无人机网络,随后连接该无人机;如果连接不上则刷新后多尝试几次,随后连接成功。

将切换为station模式的模块拖入中间空白处,并且修改为准备好的wifi路由器热点名称和密码。

随后电脑连入同一个wifi,并双击该修改好的模块,可以看到OK,drone will reboot in 3s的字样。

将无人机扩展板开关拨到上方,等待一段时间,当无人机自动连接wifi后螺旋桨将会开始转动。

在路由器管理页面找到已连接的无人机,可以找到每个无人机对应的IP地址。

## 2.2 代码介绍

通过djitellopy库的TelloSwarm连接4架无人机(指定IP地址,为保证准确可在路由器管理网址中查看),启用任务板定位功能,设定下方摄像头检测任务板。

首先将四架无人机按顺序放在飞行地图指定位置,随后通过键盘按键1触发集群起飞,起飞后,四架无人机按预设坐标target_pos(四个75cm边长的矩形顶点)循环飞行,每架无人机依次飞向下一个目标点(如Drone1: A→B→C→D→A),同时使用go_xyz_speed_mid实现高精度定位飞行(依赖飞行地图的坐标反馈)。

新建ROS2工作空间drone_ws和功能包drone_pkg,并在drone_pkg下新建程序control.py,内容如下:

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
settings = termios.tcgetattr(sys.stdin) #获取键值初始化,读取终端相关属性
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
    "192.168.1.101",
    "192.168.1.102",
    "192.168.1.103",
    "192.168.1.104"
])
swarm.connect()
tello_takeoff = False
target_pos = [[75, 75], [-75, 75], [-75, -75], [75, -75]] #4 drones

def uav_control():
    global tello_takeoff, target_pos
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
            ...

        #围绕四个点飞行
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

## 2.3 启动程序

输入以下命令安装依赖:

```bash
pip install djitellopy
```

将RDK开发板连接到无人机扩展板初始设置时设置的同一WiFi下,将无人机扩展板开关拨到上方,打开无人机电源等待机翼自动旋转。

在工作空间drone_ws中输入以下命令启动程序。(为保证准确可在路由器管理网址中查看各无人机IP并在代码中修改)

```bash
colcon build
source install/setup.bash
ros2 run drone_pkg control
```

当四架无人机都连接成功后按下按键1等待无人机起飞,随后无人机自动朝着目标前进。