# ROS记录包、回放包、提取出点云数据

## 记录所有topic

`rosbag record -a`（在想要保存的文件夹中打开terminal）

## 记录想要的topic

`rosbag record –o sensor [后面接想要录的topic名称]`

## 记录想要的topic + 命名

`rosbag record -o left /turtle1/cmd_vel /turtle1/pose`

`-o` 参数告诉 `rosbag record` 以 `left` 定义文件名字 后边的是想要记录的Topics

## 将点云数据保存为pcd文件

`rosrun pcl_ros bag_to_pcd 包 点云话题 保存pcd的路径`

## 查看pcd文件

`pcl_viewer *.pcd`