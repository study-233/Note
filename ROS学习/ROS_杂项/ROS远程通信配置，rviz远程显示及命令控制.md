# 环境

- ifconfig 查看本机ip
- 本机ip : 192.168.31.148  电脑名称 ：WP
- 远端ip : 192.168.31.84    电脑名称 ：superbrain
- 电脑名称是指ubuntu下打开终端，看到的XXXuser@YYYY中的YYYY。

## 1 . 配置本机hosts

```
sudo vim /etc/hosts
```

添加192.168.10.10 robot如下：

```
127.0.0.1       localhost
127.0.1.1       mikaa-OptiPlex-6900
192.168.31.84   superbrain

\# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
```

## 2 . 配置ROS_MASTER_URI 和 ROS_HOSTNAME

在 .bashrc 文件中添加如下配置：

```
export ROS_MASTER_URI=http://192.168.31.84:11311 #远端ip
export ROS_HOSTNAME=192.168.31.148 #本机ip
```

当然在终端直接输入也行，关闭终端就失效了

**如果需要机器人能够接受主机命令，需要在机器人上做相同配置。**

# 警告

如果后面要继续在本地使用ros，需要改动ROS_MASTER_URI和ROS_HOSTNAME

```
export ROS_HOSTNAME=localhost 
export ROS_MASTER_URI=http://localhost:11311
```

