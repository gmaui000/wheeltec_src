# 图片转换节点

将16-bit的灰度图转成8-bit的，用于rqt显示

## 使用教程

- 下载源码并编译
``` bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://gitee.com/tangyang/image_transformer.git
cd ..
rosdep install --from-paths src -i -y --rosdistro $ROS_DISTRO
catkin_make -j4
```

- 运行转换节点
```bash
source devel/setup.bash
rosrun image_transformer gray_image_transformer
```
## 节点概述

Subscriptions: 
  * /camera/ir/image [sensor_msgs/Image]

Publications: 
  * /camera/ir/image_mono8 [sensor_msgs/Image]
  * /camera/ir/image_mono8/compressed [sensor_msgs/CompressedImage]