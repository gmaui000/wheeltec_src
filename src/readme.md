# 2024.04.04

1. 打开底盘控制
       ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
2. 打开键盘控制
       ros2 run wheeltec_robot_keyboard wheeltec_keyboard
3. WEB浏览器显示摄像头
step1：
       打开usb相机
       ros2 launch usb_cam usb_cam_launch.py
       打开Astra Pro A相机
       ros2 launch ros2_astra_camera astra_pro_launch.py
step2：
       ros2 run web_video_server web_video_server
step3:
       浏览器输入：192.168.0.131:8080
4. 打开雷达
       ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py

5. 相机标定
ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.020 --no-service-check image:=/camera/color/image_raw camera:=/camera/color
ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.020 --no-service-check image:=/camera/ir/image_mono8 camera:=/camera/ir

# 代码的深入理解

1. 自带文档  3.ROS开发手册/2.STM32运动底盘开发手册_ROS教育机器人.pdf
2. https://blog.csdn.net/hbsyaaa/article/details/108186892
3. https://blog.csdn.net/lz20120808/article/details/50809397
4. https://blog.csdn.net/qq_57061492/article/details/137344534

2023.12.26更新

NFS挂载
sudo mount -t nfs 192.168.0.100:/home/wheeltec/wheeltec_ros2/ /mnt


6、简单跟随功能
① 雷达跟随
ros2 launch simple_follower_ros2 laser_follower.launch.py

② 视觉巡线
ros2 launch simple_follower_ros2 line_follower.launch.py

③ 视觉跟踪
ros2 launch simple_follower_ros2 visual_follower.launch.py

7、2D建图
①使用gmapping建图
ros2 launch slam_gmapping slam_gmapping.launch.py

②使用slam_toolbox建图
ros2 launch wheeltec_slam_toolbox online_async_launch.py

③使用cartographer建图
ros2 launch wheeltec_cartographer cartographer.launch.py

保存地图
ros2 launch wheeltec_nav2 save_map.launch.py

8、2D导航
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py

10、RTAB-MAP建图
ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab.launch.py

11、语音功能
step1：打开小车底层节点
ros2 launch wheeltec_mic_ros2 base.launch.py
step2：初始化M2麦克风阵列
ros2 launch wheeltec_mic_ros2 mic_init.launch.py

13、USB手柄控制
ros2 launch wheeltec_joy wheeltec_joy.launch.py

14、打开rviz2
注意：使用虚拟机打开rviz2

15、单独编译功能包：
如只编译 turn_on_wheeltec_robot
colcon build --packages-select turn_on_wheeltec_robot
编译全部功能包
colcon build
注意：用户修改launch文件内容后需要编译才能生效。

16.tros功能
1、物体识别：
物体识别：PC端浏览器输入http://IP:8000 即可查看图像
RDK X3启动：
sudo su
source /opt/tros/setup.bash
export CAM_TYPE=usb
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/fcosworkconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272

2、手势控制
RDK X3启动：
Step1.打开手势控制(root tros)
sudo su
rm /usr/bin/python3
ln -s /usr/bin/python3.8 /usr/bin/python3
export CAM_TYPE=usb
source /opt/tros/setup.bash 
cp -r /opt/tros/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/lib/hand_lmk_detection/config/ .
cp -r /opt/tros/lib/hand_gesture_detection/config/ .
ros2 launch gesture_control gesture_control.launch.py 

Step2.启动机器人底盘(root humble)
sudo su
rm /usr/bin/python3
ln -s /usr/bin/python3.10 /usr/bin/python3
source /opt/ros/humble/setup.bash
source /home/sunrise/wheeltec_ros2/install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py



3、人体骨骼识别:PC浏览器输入http://IP:8000 即可查看图像
Step1.启动识别算法(root tros)
sudo su 
rm /usr/bin/python3
ln -s /usr/bin/python3.8 /usr/bin/python3
source /opt/tros/setup.bash
export CAM_TYPE=usb
cp -r /opt/tros/lib/mono2d_body_detection/config/ .
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py

Step2.
sudo su 
source /opt/tros/setup.bash
rm /usr/bin/python3
ln -s /usr/bin/python3.10 /usr/bin/python3
source /home/wheeltec/wheeltec_ros2/install/setup.bash 
ros2 run wheeltec_bodyreader body_callback

Step3.启动第二第三个终端，启动机器人底盘和控制节点(root humble)
ssh -Y wheeltec@192.168.0.100 
sudo su 
source /home/wheeltec/wheeltec_ros2/install/setup.bash 
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

ROS1版本为noetic，主要功能为打开Astra S相机

ROS1调试指令：
1、切换ROS1环境，调试ROS1功能包
source /opt/ros/noetic/setup.bash
2、打开Astra S相机：
cd wheeltec_noetic/ && source devel/setup.bash
roslaunch astra_camera astra.launch
 
ROS2版本为galactic，默认相机设备为Astra S，若使用Astra Pro相机，则根据使用手册修改源码再调试。
常用调试指令如下。

1、打开ROS1桥,常用于相机节点的数据传输
终端输入指令：
       bridge

2、打开底盘控制
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py


4、打开雷达
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py

5、打开键盘控制
ros2 run wheeltec_robot_keyboard wheeltec_keyboard 

6、简单跟随功能
① 雷达跟随
ros2 launch simple_follower_ros2 laser_follower.launch.py

② 视觉巡线
ros2 launch simple_follower_ros2 line_follower.launch.py

③ 视觉跟踪
step1：打开相机
cd wheeltec_noetic/ && source devel/setup.bash
roslaunch astra_camera astra.launch
step2：打开ros1_bridge
       bridge
step3：ros2 launch simple_follower_ros2 visual_follower.launch.py

7、2D建图
①使用gmapping建图
ros2 launch slam_gmapping slam_gmapping.launch.py

②使用slam_toolbox建图
ros2 launch wheeltec_slam_toolbox online_sync.launch.py

③使用cartographer建图
ros2 launch wheeltec_cartographer cartographer.launch.py

8、2D导航
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py

9、RRT 自主探索建图
step1：ros2 launch wheeltec_slam_toolbox online_sync.launch.py
step2：ros2 launch wheeltec_robot_rrt wheeltec_rrt_slam.launch.py

10、RTAB-MAP建图
step1：打开相机
cd wheeltec_noetic/ && source devel/setup.bash
roslaunch astra_camera astra.launch
step2：打开ros1_bridge
       bridge
step3：运行RTAB-MAP建图
ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab.launch.py

11、RTAB-MAP导航
step1：打开相机
cd wheeltec_noetic/ && source devel/setup.bash
roslaunch astra_camera astra.launch
step2：打开ros1_bridge
       bridge
step3：运行RTAB-MAP导航
ros2 launch wheeltec_robot_rtab wheeltec_nav2_rtab.launch.py


12、WEB浏览器显示摄像头
step1：ros2 launch usb_cam usb_cam_launch.py(UVC)
step1：打开相机
cd wheeltec_noetic/ && source devel/setup.bash
roslaunch astra_camera astra.launch
step2：打开ros1_bridge
       bridge
step3：ros2 run web_video_server web_video_server

13、USB手柄控制
ros2 launch wheeltec_joy wheeltec_joy.launch.py

14、打开rviz2
注意：使用虚拟机打开rviz2

15、单独编译功能包：
如只编译 turn_on_wheeltec_robot
colcon build --packages-select turn_on_wheeltec_robot
编译全部功能包
colcon build
注意：用户修改launch文件内容后需要编译才能生效。


# 虚拟屏幕的创建
～～～
一、配置方法
1）安装软件
通过终端安装虚拟显示器软件，Ubuntu20.4可以用：

$ sudo apt-get install xserver-xorg-core-hwe-18.04
$ sudo apt-get install xserver-xorg-video-dummy

2）添加配置文件
在 /usr/share/X11/xorg.conf.d/ 中添加 xorg.conf 文件。
编辑 /usr/share/X11/xorg.conf.d/xorg.conf文件，内容如下：

Section "Monitor"
  Identifier "Monitor0"
  HorizSync 28.0-80.0
  VertRefresh 48.0-75.0
  Modeline "1920x1080_60.00" 172.80 1920 2040 2248 2576 1080 1081 1084 1118 -HSync +Vsync
EndSection
Section "Device"
  Identifier "Card0"
  Driver "dummy"
  VideoRam 256000
EndSection
Section "Screen"
  DefaultDepth 24
  Identifier "Screen0"
  Device "Card0"
  Monitor "Monitor0"
  SubSection "Display"
    Depth 24
    Modes "1920x1080_60.00"
  EndSubSection
EndSection

注意：虽然配置上面写了 “1920x1080”，但是实际上最大支持 “1360x768”

3）重启
重启计算机后，默认使用虚拟显示器。
使用rustdesk实现远程连接即可

注意：如果需要再用显示器，需要删除或者重命名“xorg.conf”文件
～～～