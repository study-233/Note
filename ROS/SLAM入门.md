# 建图方法

## Hector_Mapping

只使用雷达点云和障碍物配准的方法，不考虑里程计

## Gmapping

由里程计推算，激光雷达点云配准算法，修正里程计误差

#### Subscribed Topics

- tf ([tf/tfMessage](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html))

  - <the frame attached to incoming scans> frame_id→ base_link

  - base_link → odom

- scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html))

  - Laser scans to create the map from

#### Published Topics

- map_metadata([nav_msgs/MapMetaData](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html))

- map ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

- ~entropy ([std_msgs/Float64](http://docs.ros.org/en/api/std_msgs/html/msg/Float64.html))
  - 机器人定位置信度

#### Parameters

接口相关

- `base_frame` (`string`, default: `"base_link"`)
- `map_frame` (`string`, default: `"map"`)
- `odom_frame` (`string`, default: `"odom"`)

性能相关

- `map_update_interval` (`float`, default: 5.0) 地图更新间隔
- `maxRange` (`float`, default: 80.0) 激光雷达射线的最大采纳距离
- `throttle_scans` (`int`, default: 1) 激光雷达数据跳帧处理

<img src="/home/andy/Note/ROS/SLAM入门.assets/58a03069ae1a48139148cb90903454ef.jpg" alt="58a03069ae1a48139148cb90903454ef" style="zoom:80%;" />

## 在launch文件中添加启动参数

```xml
<launch>

<include file="$(find wpr_simulation)/launch/wpb_stage_slam.launch"/>

<node pkg="hector_mapping" type="hector_mapping" name="hector_mapping" output="screen">
    //添加启动参数
    <prarm name="map_update_distance_thresh" value="0.1"/>
    <prarm name="map_update_angle_thresh" value="0.1"/>
    <prarm name="map_pub_period" value="0.1"/>
</node>

<node pkg="rviz" type="rviz" name="rviz" args="-d $(find slam_pkg)/rviz/slam.rviz"/>

<node pkg="rqt_robot_steering" type="rqt_robot_steering" name="rqt_robot_steering"/>

</launch>
```



# TF变换

## tf层级结构查看

rosrun rqt_tf_tree rqt_tf_tree 

# 