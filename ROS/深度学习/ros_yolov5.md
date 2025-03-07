# 步骤

https://blog.csdn.net/weixin_64561234/article/details/130340074

https://github.com/mats-robotics/yolov5_ros

- Clone the packages to ROS workspace and install requirement for YOLOv5 submodule:

```
cd <ros_workspace>/src
git clone https://github.com/mats-robotics/detection_msgs.git
git clone --recurse-submodules https://github.com/mats-robotics/yolov5_ros.git 
cd yolov5_ros/src/yolov5
pip install -r requirements.txt # install the requirements for yolov5
```



- Build the ROS package:

```
cd <ros_workspace>
catkin build yolov5_ros # build the ROS package
```



- Make the Python script executable

```
cd <ros_workspace>/src/yolov5_ros/src
chmod +x detect.py
```

# 问题

## 1

AttributeError: module ‘importlib_metadata‘ has no attribute ‘MetadataPathFinder‘

### 原因

importlib-metadata从5.0开始不支持该功能。

### 解决办法

将版本降到5.0以下，如4.13.0
`pip install importlib-metadata==4.13.0`
或
`pip install importlib-metadata<5`

## 2

YOLOv5出现ImportError: cannot import name ‘scale_coords’ from 'utils.general’错误的解决办法
可直接将’scale_coords’替换为’scale_boxes’函数

同时将

```
from utils.general import scale_coords
替换为
```

```
from utils.general import scale_boxes
```

