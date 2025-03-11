# 标准消息包 std_msgs

## 基础类型

Bool ，Byte，Char，String，Int8、Int16、Int32、Int64，UInt8、UInt16、UInt32、Uint64，Float32、Float64，Empty

### 数组类型

ByteMultiArray，Int8MultiArray、Int16MultiArray、Int32MultiArray、Int64MultiArray，UInt8MultiArray、Uint16MultiArray、UInt32MultiArray、Uint64MultiArray，Float32MultiArray、Float64MultiArray

### 结构体类型

ColorRGBA，Duration，Time，Header，MultiArrayDimension，MultiArrayLayout

# 常规消息包 common_msgs

## 传感器消息包 sensor_msgs

### 激光雷达消息包

#### sensor_msgs/LaserScan.msg

- Header header      

- float32 angle_min  角度的起始和结束位置
  float32 angle_max    

- float32 angle_increment       扫描角度间隔

  float32 time_increment         扫描时间间隔

- float32 scan_time    扫描间隔

- float32 range_min    可以测量的最小和最大距离
  float32 range_max    

- float32[] ranges    距离
  float32[] intensities     强度

### IMU惯性测量单元消息包

#### sensor_msgs/Imu Message

- Header header
- geometry_msgs/Quaternion orientation             姿态数据 xyz旋转偏移量
  float64[9] orientation_covariance # Row major about x, y, z axes
- geometry_msgs/Vector3 angular_velocity               角速度
  float64[9] angular_velocity_covariance # Row major about x, y, z axes
- geometry_msgs/Vector3 linear_acceleration           矢量加速度
  float64[9] linear_acceleration_covariance # Row major x, y z 

使用消息包数据 需要对其协方差矩阵的第一个数值进行一个判断：

如果数值为-1 表明使用的数据是不存在的，不要去读取它

| 协方差矩阵数值 | 协方差数值 |
| :------------: | :--------: |
|    全部置 0    |    未知    |
| 第一个数为 -1  |   不存在   |

#### 默认话题

- imu/data_raw (sensor_msgs/Imu)
  - 加速度计输出的矢量加速度 和 陀螺仪输出的旋转角速度
- imu/data (sensor_msgs/Imu)
  - /imu/data raw的数据 再加上 融合后的四元数姿态描述
- imu/mag (sensor_msgs/MagneticField)
  - 磁强计输出磁强数据

## 几何消息包 geometry_msgs

## 可视化消息包 visualization_msgs

#### 标志物 visualization_msgs::Marker

http://wiki.ros.org/rviz/DisplayTypes/Marker

# 自定义消息包

## 自定义msg

### 创建消息包

1. **依赖项需要 message_generation message_runtime**

​	catkin_create_pkg qq_msgs roscpp rospy std_msgs message_generation message_runtime

2. 创建msg文件夹存放msg文件

#### 消息包格式

数据类型 变量名

### 修改配置文件

#### CMakeLists 修改添加

```camke
add_message_files(
  FILES
  Carry.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qq_msgs
 CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)
```

#### package 补全

```xml
  <buildtool_depend>catkin</buildtool_depend>
  
	//确保都存在
  <build_depend>message_generation</build_depend>
  <build_depend>message_runtime</build_depend>
  
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

	//确保都存在
  <exec_depend>message_generation</exec_depend>
  <exec_depend>message_runtime</exec_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

## 自定义消息包应用

### C++节点

1. include 头文件
2. 替换消息类型
3. CMakeList修改
   1. 在CMakeList 文件的 find_package()中 添加新消息包名称作为依赖项
   2. 添加 add_dependencies(ma_node 新消息包名称_generate_messages_cpp) 作为依赖项
4. package.xml 修改
   1. 把新消息包添加到build_depend和exec_depend

### Python节点

1. include 头文件
2. 替换消息类型
3. CMakeList修改
   1. 在CMakeList 文件的 find_package()中 添加新消息包名称作为依赖项
4. package.xml 修改
   1. 把新消息包添加到build_depend和exec_depend
