# 话题通信

- 持续通讯
- 数据中的数据叫做Message
- Message通常会按照一定的频率**持续不断**地发送

发送方——Publisher

接收方——Subsciber

## 发布话题

1. 向NodeHandle申请初始化节点
2. 创建发布对象pub
3. 开启while循环 使用pub对象发布消息包

### C++

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"chao_node");
    printf("Hello World!\n");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("Topic1",10);
    
    ros::Rate loop_rate(10);

    while(ros::ok()){
        printf("I'm here\n");
        std_msgs::String msg;
        msg.data = "你好";
        pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}
```

### Python

```python
#!/usr/bin/env python3
#coding=utf-8

import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node("chao_node")
    rospy.logwarn("hello")

    pub = rospy.Publisher("Topic1",String,queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("--------")
        msg = String()
        msg.data = "callback"
        pub.publish(msg)
        rate.sleep()
```



## 订阅话题

1. 确定话题名称和消息类型
2. 导入消息类型对应的头文件
3. 通过NodeHandler订阅话题 设置消息接收回调函数
4. 定义回调函数，处理消息包
5. main函数中需要执行 ros::spinOnce()   让回调函数能够响应接收到的消息包

### C++

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

void chao_callback(std_msgs::String msg){
    ROS_INFO(msg.data.c_str());		//带有时间戳
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"ma_node");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("Topic1", 10, chao_callback);
    
    while(ros::ok()){
        ros::spinOnce();	//必须有
    }
    return 0;
}
```

### Python

```python
#!/usr/bin/env python3
#coding=utf-8

import rospy
from std_msgs.msg import String

def chao_callback(msg):
    rospy.loginfo(msg.data)

if __name__ == "__main__":
    rospy.init_node("ma_node")

    sub = rospy.Subscriber("Topic1",String,chao_callback,queue_size=10)

    rospy.spin()
```

# ROS 调试常用工具

|            命令             |           作用           |
| :-------------------------: | :----------------------: |
|        rostopic list        |   列出当前所有的topic    |
|  rostopic info topic_name   | 显示某个topic的属性信息  |
|  rostopic echo topic_name   |   显示某个topic的内容    |
| rostopic pub topic_name ... |   向某个topic发布内容    |
|   rostopic bw topic_name    |   查看某个topic的带宽    |
|   rostopic hz topic_name    |   查看某个topic的频率    |
|  rostopic find topic_type   |   查找某个类型的topic    |
|  rostopic type topic_name   | 查看某个topic的类型(msg) |
|          rqt_graph          |      图形化话题工具      |