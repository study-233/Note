# Launch文件

- 可以使用roslaunch指令一次启动多个节点
- 在launch文件中 为节点添加 output=“screen” 可以让节点信息输出在终端中
  - ROS_WARN不受该属性控制
- 在launch文件中 为节点添加 launch-prefix="gnome-terminal -e" 可以让节点单独运行在一个独立终端中



- c++文件

```xml
<launch>

    <node pkg="ssr_pkg" type="yao_node" name="yao_node"/>
    
    <node pkg="ssr_pkg" type="chao_node" name="chao_node" launch-prefix="gnome-terminal -e"/>
    
    <node pkg="atr_pkg" type="ma_node" name="ma_node" output="screen"/>

</launch>
```

- py文件(type需要加上后缀)

```xml
<launch>

    <node pkg="ssr_pkg" type="yao_node.py" name="yao_node"/>
    
    <node pkg="ssr_pkg" type="chao_node.py" name="chao_node" launch-prefix="gnome-terminal -e"/>
    
    <node pkg="atr_pkg" type="ma_node.py" name="ma_node" output="screen"/>

</launch>
```

# launch文件中嵌套launch启动

- 使用include 即可  file后为launch文件路径
- $(find 软件包名) = 软件包路径

```xml
<include file="$(find wpr_simulation)/launch/wpb_stage_slam.launch"/>
```

