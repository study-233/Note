# TF坐标变换概念

在ROS中用于实现不同的坐标系之间的点和向量的转换

tf2 常用功能包

| tf2_geomerty_msgs | 将ROS消息转换为tf2消息                              |
| ----------------- | --------------------------------------------------- |
| tf2               | 封装了坐标变换的常用消息（四元数）                  |
| tf2_ros           | 为tf2提供了roscpp和rospy绑定，封装了坐标变换常用API |

# 坐标msg消息

## 坐标转换消息 geometry_msgs/TransformStamped 

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

## 坐标点 geometry_msgs/PointStamped

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

- ### C++

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

- ### Python

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

- ### C++

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

- ### Python

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

## 发布方

- ### C++

```cpp
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/*
    发布方：订阅乌龟位姿信息，转换为相对于窗体的坐标关系

*/


void doPose(const turtlesim::Pose::ConstPtr& pose){

    static tf2_ros::TransformBroadcaster pub;

    geometry_msgs::TransformStamped tfs;

    tfs.header.frame_id = "world";
    tfs.child_frame_id = "turtle1";
    tfs.header.stamp = ros::Time::now();
    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0;

    //根据欧拉角转换
    tf2::Quaternion qtn;    //创建四元数对象
    //设置欧拉角
    qtn.setRPY(0,0,pose->theta);  //欧拉角单位是弧度

    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();  

    pub.sendTransform(tfs);
}

int main(int argc, char *argv[]){
    
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "dynamic_pub_node");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/turtle1/pose",10,doPose);

    ros::spin();

    return 0;
}

```

- ### Python

```python
#! /usr/bin/env python3
import rospy
import tf2_ros.transform_broadcaster
from turtlesim.msg import Pose
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations

def doPose(pose):

    pub = tf2_ros.TransformBroadcaster()

    tfs = TransformStamped()

    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time.now()
    tfs.child_frame_id = "turtle1"

    tfs.transform.translation.x = pose.x
    tfs.transform.translation.y = pose.y
    tfs.transform.translation.z = 0

    qtn = tf.transformations.quaternion_from_euler(0,0,pose.theta)

    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]

    pub.sendTransform(tfs)

if __name__ == "__main__":

    rospy.init_node("dynamic_pub_node")

    sub = rospy.Subscriber("/turtle1/pose", Pose, doPose, queue_size=10)

    rospy.spin()
```

## 订阅方

- ### C++

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv,"dynamic_sub_node");

    ros::NodeHandle nh;

    //创建 buffer 缓存
    tf2_ros::Buffer buffer;

    //创建订阅对象
    tf2_ros::TransformListener listener(buffer);

    //组织坐标点数据

    geometry_msgs::PointStamped ps;

    //动态坐标系转换需要改动时间戳
    //原因：ROS会比对坐标点时间戳和坐标关系时间戳，判断时间值是否接近，设置0让ROS忽略
    ps.header.stamp = ros::Time(0.0);
    ps.header.frame_id = "turtle1";
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
                原因：订阅数据是个耗时操作，可能在调用 transfrom 转换函数时，坐标系的相对关系没有订阅到
                解决：
                    方案1：在调用转换函数前执行休眠 ros::Duration(1).sleep();
                    方案2：进行异常处理
        */
        try{
            ps_out = buffer.transform(ps, "world");
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

- ### Python

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

    rospy.init_node("dynamic_sub_node")

    buffer = tf2_ros.Buffer()

    sub = tf2_ros.TransformListener(buffer)

    ps = tf2_geometry_msgs.PointStamped()
    #注意相对于静态变换 需要修改时间戳
    ps.header.stamp = rospy.Time()
    ps.header.frame_id = "world"
    ps.point.x = 2.0
    ps.point.y = 3.0
    ps.point.z = 5.0

    #转换逻辑实现
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            ps_out = buffer.transform(ps, "turtle1")
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

# 多坐标变换

## **需求描述:**

现有坐标系统，父级坐标系统 world,下有两子级系统 son1，son2，son1 相对于 world，以及 son2 相对于 world 的关系是已知的，求 son1原点在 son2中的坐标，又已知在 son1中一点的坐标，要求求出该点在 son2 中的坐标

## **实现分析:**

1. 首先，需要发布 son1 相对于 world，以及 son2 相对于 world 的坐标消息
2. 然后，需要订阅坐标发布消息，并取出订阅的消息，借助于 tf2 实现 son1 和 son2 的转换
3. 最后，还要实现坐标点的转换

## **实现流程: **C++ 与 Python 实现流程一致

1. 新建功能包，添加依赖
2. 创建坐标相对关系发布方(需要发布两个坐标相对关系)
3. 创建坐标相对关系订阅方
4. 执行

### 创建功能包

创建项目功能包依赖于 tf2、tf2_ros、tf2_geometry_msgs、roscpp rospy std_msgs geometry_msgs

### 发布方

```xml
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="son1" args="0.2 0.8 0.3 0 0 0 /world /son1" output="screen" />
    <node pkg="tf2_ros" type="static_transform_publisher" name="son2" args="0.5 0 0 0 0 0 /world /son2" output="screen" />
</launch>
```

### 订阅方

- #### C++

```cpp
/*

需求:
    现有坐标系统，父级坐标系统 world,下有两子级系统 son1，son2，
    son1 相对于 world，以及 son2 相对于 world 的关系是已知的，
    求 son1 与 son2中的坐标关系，又已知在 son1中一点的坐标，要求求出该点在 son2 中的坐标
实现流程:
    1.包含头文件
    2.初始化 ros 节点
    3.创建 ros 句柄
    4.创建 TF 订阅对象
    5.解析订阅信息中获取 son1 坐标系原点在 son2 中的坐标
      解析 son1 中的点相对于 son2 的坐标
    6.spinOnce()

*/
//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"  

int main(int argc, char *argv[])
{   setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"sub_frames");
    // 3.创建 ros 句柄
    ros::NodeHandle nh;
    // 4.创建 TF 订阅对象
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);
    // 5.解析订阅信息中获取 son1 坐标系原点在 son2 中的坐标
    ros::Rate r(1);
    while (ros::ok())
    {
        try
        {
        //   解析 son1 中的点相对于 son2 的坐标
            /*
                A 相对于 B 的坐标系关系

                参数1：目标坐标系   B
                参数2：源坐标系     A
                参数3：ros::Time(0) 取间隔最短的两个坐标关系帧计算相对关系
                返回值：geometry_msgs::TransformStamped 源相对于目标坐标系的相对关系
            
            */
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("son2","son1",ros::Time(0));
            ROS_INFO("Son1 相对于 Son2 的坐标关系:父坐标系ID=%s",tfs.header.frame_id.c_str());
            ROS_INFO("Son1 相对于 Son2 的坐标关系:子坐标系ID=%s",tfs.child_frame_id.c_str());
            ROS_INFO("Son1 相对于 Son2 的坐标关系:x=%.2f,y=%.2f,z=%.2f",
                    tfs.transform.translation.x,
                    tfs.transform.translation.y,
                    tfs.transform.translation.z
                    );

            // 坐标点解析
            geometry_msgs::PointStamped ps;
            ps.header.frame_id = "son1";
            ps.header.stamp = ros::Time::now();
            ps.point.x = 1.0;
            ps.point.y = 2.0;
            ps.point.z = 3.0;

            geometry_msgs::PointStamped psAtSon2;
            psAtSon2 = buffer.transform(ps,"son2");
            ROS_INFO("在 Son2 中的坐标:x=%.2f,y=%.2f,z=%.2f",
                    psAtSon2.point.x,
                    psAtSon2.point.y,
                    psAtSon2.point.z
                    );
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("异常信息:%s",e.what());
        }


        r.sleep();
        // 6.spin
        ros::spinOnce();
    }
    return 0;
}
```

- #### Python

```python
#!/usr/bin/env python
"""  
    需求:
        现有坐标系统，父级坐标系统 world,下有两子级系统 son1，son2，
        son1 相对于 world，以及 son2 相对于 world 的关系是已知的，
        求 son1 与 son2中的坐标关系，又已知在 son1中一点的坐标，要求求出该点在 son2 中的坐标
    实现流程:   
        1.导包
        2.初始化 ROS 节点
        3.创建 TF 订阅对象
        4.调用 API 求出 son1 相对于 son2 的坐标关系
        5.创建一依赖于 son1 的坐标点，调用 API 求出该点在 son2 中的坐标
        6.spin

"""
# 1.导包
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import PointStamped

if __name__ == "__main__":

    # 2.初始化 ROS 节点
    rospy.init_node("frames_sub_p")
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        try:
        # 4.调用 API 求出 son1 相对于 son2 的坐标关系
            #lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            tfs = buffer.lookup_transform("son2","son1",rospy.Time(0))
            rospy.loginfo("son1 与 son2 相对关系:")
            rospy.loginfo("父级坐标系:%s",tfs.header.frame_id)
            rospy.loginfo("子级坐标系:%s",tfs.child_frame_id)
            rospy.loginfo("相对坐标:x=%.2f, y=%.2f, z=%.2f",
                        tfs.transform.translation.x,
                        tfs.transform.translation.y,
                        tfs.transform.translation.z,
            )
        # 5.创建一依赖于 son1 的坐标点，调用 API 求出该点在 son2 中的坐标
            point_source = PointStamped()
            point_source.header.frame_id = "son1"
            point_source.header.stamp = rospy.Time.now()
            point_source.point.x = 1
            point_source.point.y = 1
            point_source.point.z = 1

            point_target = buffer.transform(point_source,"son2",rospy.Duration(0.5))

            rospy.loginfo("point_target 所属的坐标系:%s",point_target.header.frame_id)
            rospy.loginfo("坐标点相对于 son2 的坐标:(%.2f,%.2f,%.2f)",
                        point_target.point.x,
                        point_target.point.y,
                        point_target.point.z
            )

        except Exception as e:
            rospy.logerr("错误提示:%s",e)


        rate.sleep()
    # 6.spin    
    # rospy.spin()
```

# tf层级结构查看

rosrun rqt_tf_tree rqt_tf_tree 
