# 服务通信

- 适用于对实时性有要求，有逻辑处理的应用场景
- 以请求响应的方式实现节点间通信

服务端——Server

客户端——Client

## 服务器端实现

1. 导入消息包中的srv
   1. c++/python 导包有所不同

2. 向NodeHandle申请初始化节点
3. 创建服务对象 server
4. 创建回调函数处理request，返回response

### C++

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <qq_msgs/AddInts.h>

bool donums(qq_msgs::AddInts::Request &request,
            qq_msgs::AddInts::Response &response){
    int num1 = request.num1;
    int num2 = request.num2;
    ROS_INFO("Get num1 = %d,num2 = %d", num1, num2);
    int sum = num1 + num2;
    response.sum = sum;
    ROS_INFO("Back sum = %d", sum);
    return true;
}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"test_server_node");

    ros::NodeHandle nh;
    
    ros::ServiceServer server = nh.advertiseService("AddInts",donums);

    ROS_INFO("Server Online");
    ros::spin();
}

```

### Python

```python
#!/usr/bin/env python3
#coding=utf-8

import rospy
from std_msgs.msg import String

#注意此处导包与 msg 有所不同 将下面的类型全部导入
from qq_msgs.srv import *	


def doNum(requset):
    num1 = requset.num1
    num2 = requset.num2

    sum = num1 + num2
    
    response = AddIntsResponse()

    response.sum = sum

    rospy.loginfo("Get num1:%d num2:%d",num1,num2)
    return response

if __name__ == "__main__":
    rospy.init_node("demo_server_node")

    server = rospy.Service("addInts", AddInts, doNum)
    rospy.loginfo("Server online")
    rospy.spin()
```



## 客户端实现

1. 向NodeHandle申请初始化节点
2. 创建客户端对象 client
3. 提交请求处理响应

### C++

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <qq_msgs/AddInts.h>


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"test_client_node");

    ros::NodeHandle nh;
    
    ros::ServiceClient client = nh.serviceClient<qq_msgs::AddInts>("AddInts");

    //提交请求并处理响应

    qq_msgs::AddInts ai;
    ai.request.num1 = 10;
    ai.request.num2 = 100;

    bool flag = client.call(ai);    
    if(flag){
        int sum = ai.response.sum;
        ROS_INFO("Get sum = %d",sum);
    }
    else{
        ROS_INFO("Fail");
    }
    ros::spin();
}

```

#### 客户端优化

实现参数动态提交

`rosrun ros_pkg ros_node 12 34`

```cpp
int main(int argc, char *argv[])
{

    //优化实现，获取命令中参数
    setlocale(LC_ALL,"");

    if(argc != 3){
        ROS_INFO("参数个数不对");
        return 1;
    }

    ros::init(argc,argv,"test_client_node");

    ros::NodeHandle nh;
    
    ros::ServiceClient client = nh.serviceClient<qq_msgs::AddInts>("AddInts");

    //提交请求并处理响应

    qq_msgs::AddInts ai;
    ai.request.num1 = atoi(argv[1]);
    ai.request.num2 = atoi(argv[2]);
```

### Python

```python
#!/usr/bin/env python3
#coding=utf-8

import rospy
from std_msgs.msg import String
from qq_msgs.srv import *

if __name__ == "__main__":
    rospy.init_node("demo_client_node")

    client = rospy.ServiceProxy("addInts",AddInts)

    response = client.call(1,2)

    rospy.loginfo("Get %d",response.sum)
```

#### 客户端优化

```python
#!/usr/bin/env python3
#coding=utf-8

import rospy
from std_msgs.msg import String
from qq_msgs.srv import *
#导入sys包
import sys


if __name__ == "__main__":
	#判断参数个数
    if len(sys.argv) != 3:
        rospy.loginfo("参数个数不对")
        sys.exit(1)
    
    rospy.init_node("demo_client_node")

    client = rospy.ServiceProxy("addInts",AddInts)
	#传入参数
    num1 = int(sys.argv[1])
    num2 = int(sys.argv[2])
    response = client.call(num1,num2)

    rospy.loginfo("Get %d",response.sum)
```



### 挂起客户端，等待服务端

二者皆可使用

#### C++

- `client.waitForExistence();`
- `ros::service::waitForService(“AddInts”);`

#### Python

- `client.wait_for_service()`
- `rospy.wait_for_service("addInts")`

## 操作指令

| rosservice 命令 |           作用           |
| :-------------: | :----------------------: |
| rosservice list |       显示服务列表       |
| rosservice info |       打印服务信息       |
| rosservice type |       打印服务类型       |
| rosservice uri  |    打印服务ROSRPC uri    |
| rosservice find |    按服务类型查找服务    |
| rosservice call | 使用所提供的args调用服务 |
| rosservice args |       打印服务参数       |