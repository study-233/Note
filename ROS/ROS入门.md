# 记得刷新环境变量

- 解决方法一：

  在运行相关launch文件之前输入“source devel/setup.bash”刷新环境变量

- 解决方法二：

  1- 在home目录下打开终端

  2- gedit ~/.bashrc

  3- 在文件的最末尾输入你的工作空间setup.bash文件的绝对地址

  4- source /home/username/xxx/devel/setup.bash  //username为你电脑的名称、xxx为你工作空间的名字，这个只是个例子，实际的路径要根据实际的绝对路径填写

  5- source ~/.bashrc //保存文件后使其生效

## APT源 网站

index.ros.org

## scripts 

存放python脚本文件 

## 将source指令添加到.bashrc脚本中

```bash
gedit ~/.bashrc

//在文件中添加

source ~/ROS_WS/catkin_ws/devel/setup.bash
```

## vscode设置快捷编译

```json
//task.json 修改为
{
	"version": "2.0.0",
	"tasks": [
		{
			"type": "catkin_make",
			"args": [
				"--directory",		//在这个下面的目录里执行
				"/home/andy/ROS_WS/catkin_ws",
				"-DCMAKE_BUILD_TYPE=RelWithDebInfo"
			],
			"problemMatcher": [
				"$catkin-gcc"
			],
			"group": {"kind":"build","isDefault": true},
			"label": "catkin_make: build"
		}
	]
}
```

## 意外没有关闭roscore

使用以下指令删除roscore进程

killall -9 roscore

killall -9 rosmaster

## 启动完Gazebo后，关闭，再次启动出现如下报错：

```
[gazebo-1] process has died [pid 10999, exit code 255, cmd /opt/ros/melodic/lib/gazebo_ros/gzserver -e ode worlds/empty.world __name:=gazebo __log:=/home/will/.ros/log/5ee74f98-0ca5-11ec-9c23-b8ca3a8092a4/gazebo-1.log].
log file: /home/will/.ros/log/5ee74f98-0ca5-11ec-9c23-b8ca3a8092a4/gazebo-1*.log
```

​	原因： 关闭的过程中有些Gazebo的程序没有完全关闭，导致再次启动时，无法正常开启。
所以，解决方法也很简单，只要把gazebo的相关程序关闭即可。
​	用如下指令：
`killall gzserver`

## ROS 指令

- catkin_make

- catkin_create_pkg <pkgname> <依赖列表>

  ```bash
  catkin_create_pkg ssr_pkg rospy roscpp std_msgs
  ```

  

## 如题报错：

terminate called after throwing an instance of 'ros::InvalidNameException'
  what():  Character [ ] at element [2] is not valid in Graph Resource Name [my control].  Valid characters are a-z, A-Z, 0-9, / and _.
已放弃 (核心已转储)

解决：

一般是因为节点的cpp文件代码或文件名中包含空格或其他所谓的非法字符导致的，删去或替换成下划线即可



### ros::ok

在代码中用 ros::ok 来代替 true ，可以响应外部信号

```
// Crtl+C 无法停止
while(true){

}

// Crtl+C 停止
while(ros::ok){

}
```

