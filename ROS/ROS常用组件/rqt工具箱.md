# 启动

- 方式1:`rqt`
- 方式2:`rosrun rqt_gui rqt_gui`

# 常用插件：

## rqt_graph

- **简介: **可视化显示计算图
- **启动: **可以在 rqt 的 plugins 中添加，或者使用`rqt_graph`启动

## rqt_console

- **简介:** rqt_console 是 ROS 中用于显示和过滤日志的图形化插件
- **准备: **编写 Node 节点输出各个级别的日志信息

## rqt_plot

- **简介:** 图形绘制插件，可以以 2D 绘图的方式绘制发布在 topic 上的数据
- **准备: **启动 turtlesim 乌龟节点与键盘控制节点，通过 rqt_plot 获取乌龟位姿
- **启动: **可以在 rqt 的 plugins 中添加，或者使用`rqt_plot`启动

### rqt_bag

- **简介:** 录制和重放 bag 文件的图形化插件
- **准备:** 启动 turtlesim 乌龟节点与键盘控制节点
- **启动: **可以在 rqt 的 plugins 中添加，或者使用`rqt_bag`启动