# ROS 入门问题

## 1. 环境变量刷新

### 方法一：临时刷新
在运行相关launch文件之前，输入以下命令刷新环境变量：
```bash
source devel/setup.bash
```

### 方法二：永久刷新
1. 在home目录下打开终端。
2. 编辑`.bashrc`文件：
   ```bash
   gedit ~/.bashrc
   ```
3. 在文件末尾添加工作空间`setup.bash`文件的绝对路径，例如：
   ```bash
   source /home/username/xxx/devel/setup.bash
   ```
   - `username`为你的电脑用户名。
   - `xxx`为你的工作空间名称。
4. 保存文件后使其生效：
   ```bash
   source ~/.bashrc
   ```

---

## 2. APT源
ROS的APT源网站为：
```
index.ros.org
```

---

## 3. 脚本文件存放
- `scripts`目录用于存放Python脚本文件。

---

## 4. 将`source`指令添加到`.bashrc`脚本中
1. 打开`.bashrc`文件：
   ```bash
   gedit ~/.bashrc
   ```
2. 在文件中添加：
   ```bash
   source ~/ROS_WS/catkin_ws/devel/setup.bash
   ```

---

## 5. VSCode设置快捷编译
修改`task.json`文件如下：
```json
{
	"version": "2.0.0",
	"tasks": [
		{
			"type": "catkin_make",
			"args": [
				"--directory",
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

---

## 6. 意外未关闭`roscore`
使用以下指令删除`roscore`进程：
```bash
killall -9 roscore
killall -9 rosmaster
```

---

## 7. Gazebo启动问题
### 问题描述：
启动Gazebo后关闭，再次启动时出现如下报错：
```
[gazebo-1] process has died [pid 10999, exit code 255, cmd /opt/ros/melodic/lib/gazebo_ros/gzserver -e ode worlds/empty.world __name:=gazebo __log:=/home/will/.ros/log/5ee74f98-0ca5-11ec-9c23-b8ca3a8092a4/gazebo-1.log].
log file: /home/will/.ros/log/5ee74f98-0ca5-11ec-9c23-b8ca3a8092a4/gazebo-1*.log
```
### 解决方法：
关闭Gazebo时，有些程序未完全关闭，导致再次启动时无法正常开启。使用以下指令关闭相关程序：
```bash
killall gzserver
```

---

## 8. ROS 常用指令
- **编译工作空间**：
  ```bash
  catkin_make
  ```
- **创建ROS包**：
  ```bash
  catkin_create_pkg <pkgname> <依赖列表>
  ```
  例如：
  ```bash
  catkin_create_pkg ssr_pkg rospy roscpp std_msgs
  ```

---

## 9. 常见报错及解决
### 报错信息：
```
terminate called after throwing an instance of 'ros::InvalidNameException'
  what():  Character [ ] at element [2] is not valid in Graph Resource Name [my control].  Valid characters are a-z, A-Z, 0-9, / and _.
已放弃 (核心已转储)
```
### 解决方法：
通常是因为节点的cpp文件代码或文件名中包含空格或其他非法字符。删除或替换为下划线即可。

---

## 10. `ros::ok`的使用
在代码中使用`ros::ok`代替`true`，可以响应外部信号（如Ctrl+C）：
```cpp
// Crtl+C 无法停止
while(true){

}

// Crtl+C 可以停止
while(ros::ok()){

}
```

