# 创建工作空间

1. 创建工作空间 catkin_ws

   - 创建src文件，放置功能包源码：

     `mkdir -p ~/catkin_ws/src`

   - 进入src文件夹： 

     `cd ~/catkin_ws/src`

   - 初始化文件夹：

      `catkin_init_workspace`

   这样就在src文件中创建了一个 CMakeLists.txt 的文件，目的是告诉系统，这个是ROS的工作空间。

2. 编译工作空间 catkin_make
   所有编译工作都要在catkin_ws文件夹下编译：

   `cd ~/catkin_ws/`

   编译，编译完成后，会发现catkin_ws中多了两个文件 build 和 develcatkin_make

3. 设置环境变量
   在第1篇中，我们介绍了设置环境变量，那个是将整个ros系统的环境变量设置到bash脚本中，现在我们需要把我们工作空间的环境变量设置到bash中。

   `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
    让上面的配置在当前的终端生效：

   `source ~/.bashrc`

