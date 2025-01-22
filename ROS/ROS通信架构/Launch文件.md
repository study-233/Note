# Launch文件

- 可以使用roslaunch指令一次启动多个节点
- 在launch文件中 为节点添加 output=“screen” 可以让节点信息输出在终端中
  - ROS_WARN不受该属性控制
- 在launch文件中 为节点添加 launch-prefix="gnome-terminal -e" 可以让节点单独运行在一个独立终端中

## 格式

```xml
<launch>    <!--根标签-->
    <node>    <!--需要启动的node及其参数-->
    <include>    <!--包含其他launch-->
    <machine>    <!--指定运行的机器-->
    <env-loader>    <!--设置环境变量-->
    <param>    <!--定义参数到参数服务器-->
    <rosparam>    <!--启动yaml文件参数到参数服务器-->
    <arg>    <!--定义变量-->
    <remap>    <!--设定参数映射-->
    <group>    <!--设定命名空间-->
</launch>    <!--根标签-->
```

## 示例

### 启动c++文件节点

```xml
<launch>

    <node pkg="ssr_pkg" type="yao_node" name="yao_node"/>
    
    <node pkg="ssr_pkg" type="chao_node" name="chao_node" launch-prefix="gnome-terminal -e"/>
    
    <node pkg="atr_pkg" type="ma_node" name="ma_node" output="screen"/>

</launch>
```

### 启动py文件节点(type需要加上后缀)

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

