# 概念

- action 类似于服务通信的实现，实现模型也包含请求和响应，不同的是在请求和响应的过程中，服务端可以连续的反馈当前任务进度，客户端可以接收反馈并且可以取消任务

# 自定义action文件

## 创建功能包/导入依赖

catkin_create_pkg demo_action rospy roscpp std_msgs actionlib actionlib_msgs

## 文件格式

- 目标数据变量
- 最终响应变量
- 连续反馈变量

```action
int32 num
---
int32 result
---
float64 progress_bar
```

## 配置文件

CMakeLists

```cmake
add_action_files(
  FILES
  AddInts.action
)
generate_messages(
  DEPENDENCIES
  actionlib_msgs  
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_action
 CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)
```

