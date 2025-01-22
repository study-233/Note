# srv文件

一个srv文件描述一项服务。它包含两个部分：请求和响应

msg文件存放在package的msg目录下，srv文件则存放在srv目录下。

类似msg文件，srv文件是用来描述服务(service)数据类型的，service通信的数据格式就定义在*.srv格式的文件中。它声明了一个服务类型，包括请求(request)和响应（reply）两部分。

## 消息包格式

```srv
bool start_detect     #client请求服务格式
---                   #分界线
my_pkg/HumanPose[] pose_data    #server返回数据格式
```

## 创建消息包

1. **依赖项需要 message_generation message_runtime**

​	catkin_create_pkg qq_msgs roscpp rospy std_msgs message_generation message_runtime

2. 创建srv文件夹存放srv文件

## 修改配置文件

### CMakeLists 修改添加

```camke
add_message_files(
  FILES
  AddInts.srv
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

### package 补全

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

## 操作指令

|   rossrv 命令   |       作用       |
| :-------------: | :--------------: |
|   rossrv show   |   显示服务描述   |
|   rossrv list   |   列出所有服务   |
|   rossrv md5    |  显示服务md5sum  |
| rossrv package  |  列出包中的服务  |
| rossrv packages | 列出包含服务的包 |