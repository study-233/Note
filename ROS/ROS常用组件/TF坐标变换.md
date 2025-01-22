# TF坐标变换概念

在ROS中用于实现不同的坐标系之间的点和向量的转换

tf2 常用功能包

| tf2_geomerty_msgs | 将ROS消息转换为tf2消息                              |
| ----------------- | --------------------------------------------------- |
| tf2               | 封装了坐标变换的常用消息（四元数）                  |
| tf2_ros           | 为tf2提供了roscpp和rospy绑定，封装了坐标变换常用API |

# 坐标msg消息

## geometry_msgs/TransformStamped 

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id	#坐标id
string child_frame_id	#子坐标id
geometry_msgs/Transform transform	#坐标信息
  geometry_msgs/Vector3 translation		#偏移量
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation		#四元数
    float64 x
    float64 y
    float64 z
    float64 w
```

## geometry_msgs/PointStamped

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id	#所属坐标系id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z
```

# 静态坐标变换

## 发布方

1. 初始化ROS节点
2. 创建静态坐标转换广播器
3. 创建坐标系信息
4. 广播器发布坐标信息
5. spin()

### C++

```cpp
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv,"static_node");
    ros::NodeHandle nh;

    tf2_ros::StaticTransformBroadcaster pub;

    geometry_msgs::TransformStamped tfs;

    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "base_link";   //相对坐标系关系中被参考
    tfs.child_frame_id = "laser";
    tfs.transform.translation.x = 0.2;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.5;

    //根据欧拉角转换
    tf2::Quaternion qtn;    //创建四元数对象
    //设置欧拉角
    qtn.setRPY(0,0,0);  //欧拉角单位是弧度

    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();     

    pub.sendTransform(tfs);

    ros::spin();

    return 0;
}

```

### Python

```python
#! /usr/bin/env python3
#coding=utf-8
import rospy
import tf.transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf

if __name__ == "__main__":

    rospy.init_node("static_pub_node")

    pub = tf2_ros.StaticTransformBroadcaster()

    tfs = TransformStamped()

    tfs.header.frame_id = "base_link"
    tfs.header.stamp = rospy.Time.now()
    tfs.child_frame_id = "laser"

    tfs.transform.translation.x = 0.2
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.5
    
    qtn = tf.transformations.quaternion_from_euler(0,0,0)

    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]

    pub.sendTransform(tfs)

    rospy.spin()


```



## 订阅方

1. 编码初始化，NodeHandle （必须）

2. 创建订阅对象 ----> 订阅坐标系相对关系

3. 组织坐标点数据

4. 转换算法，调用TF内置实现

    1. 调用时必须包含头文件  tf2_geometry_msgs/tf2_geometry_msgs.h

    2. PS:运行时存在问题，抛出异常：base_link 不存在

        1. 原因：订阅数据是个耗时操作，可能在调用 transfrom 转换函数时，坐标系的相对关系没有订阅到

        2. 解决：

           1. 方案1：在调用转换函数前执行休眠 ros::Duration(1).sleep();

           1. 方案2：进行异常处理


5. 输出

### C++

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv,"static_sub_node");

    ros::NodeHandle nh;

    //创建 buffer 缓存
    tf2_ros::Buffer buffer;

    //创建订阅对象
    tf2_ros::TransformListener listener(buffer);

    //组织坐标点数据
    geometry_msgs::PointStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "laser";
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;

    // ros::Duration(1).sleep();

    //转换算法，调用TF内置实现
    ros::Rate rate(10);
    while(ros::ok()){

        //核心代码

        geometry_msgs::PointStamped ps_out;
        
        /*
            调用时必须包含头文件 tf2_geometry_msgs/tf2_geometry_msgs.h

            PS:运行时存在问题，抛出异常：base_link 不存在
                原因：订阅数据是个耗时操作，可能在调用 transfrom 转换函数时，
                	 坐标系的相对关系没有订阅到
                解决：
                    方案1：在调用转换函数前执行休眠 ros::Duration(1).sleep();
                    方案2：进行异常处理
        */
        try{
            ps_out = buffer.transform(ps, "base_link");
            ROS_INFO("转换后坐标值: %.2f %.2f %.2f, 参考的坐标系 %s",
                        ps_out.point.x,
                        ps_out.point.y,
                        ps_out.point.z,
                        ps.header.frame_id.c_str()
                        );
        }
        catch(const std::exception& e){
            // std::cerr << e.what() << '\n';
            ROS_INFO("异常错误:%s",e.what());
        }
       
        
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

```

### Python

```python
#! /usr/bin/env python3
#coding=utf-8
import rospy
import tf.transformations
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import tf

if __name__ == "__main__":

    rospy.init_node("static_sub_node")

    buffer = tf2_ros.Buffer()

    sub = tf2_ros.TransformListener(buffer)

    ps = tf2_geometry_msgs.PointStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "laser"
    ps.point.x = 2.0
    ps.point.y = 3.0
    ps.point.z = 5.0

    #转换逻辑实现
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            ps_out = buffer.transform(ps, "base_link")
            rospy.loginfo("转换后的坐标为 (%.2f,%.2f,%.2f),参考坐标系%s",
                        ps_out.point.x,
                        ps_out.point.y,
                        ps_out.point.z,
                        ps_out.header.frame_id
                        )
        except Exception as e:
            rospy.logwarn("错误提示:%s",e)

        rate.sleep()
        


```

## 补充知识

```
rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏航角度 y俯仰角度 x翻滚角度 父级坐标系 子级坐标系 
```

# 动态坐标系变换



# tf层级结构查看

rosrun rqt_tf_tree rqt_tf_tree 
