# 问题

rk3588:failed to open rknpu module, need to insmod rknpu dirver!

没有安装驱动模块

## 模块安装

假设要加载的驱动程序模块名为SHT21.ko

### 1、加载驱动模块

**方法一**

- 进入SHT21.ko驱动模块文件所在的目录，然后直接`insmod SHT21.ko`即可

**方法二**

- 将`SHT21.ko`文件拷贝到`/lib/module/#uname -r#/`目录下，这里，`#uname -r#`意思是，在终端中输入`uname -r`后显示的内核版本及名称，例如`mini2440`中`#uname -r#`就是`2.6.32.2-FriendlyARM`。
- 然后`depmod`（会在`/lib/modules/#uname -r#/`目录下生成`modules.dep`和`modules.dep.bb`文件，表明模块的依赖关系）
- 最后`modprobe SHT21`（注意这里无需输入`.ko`后缀）即可。

**两种方法的区别**
`modprobe`和`insmod`类似，都是用来动态加载驱动模块的，区别在于`modprobe`可以解决`load module`时的依赖关系，它是通过`/lib/modules/#uname -r/modules.dep(.bb)`文件来查找依赖关系的；而`insmod`不能解决依赖问题。

也就是说，如果你确定你要加载的驱动模块不依赖其他驱动模块的话，既可以`insmod`也可以`modprobe`，当然`insmod`可以在任何目录下执行，更方便一些。而如果你要加载的驱动模块还依赖其他`ko`驱动模块的话，就只能将模块拷贝到上述的特定目录，`depmod`后再`modprobe`。

### 2、查看已加载的驱动模块列表

在任何目录下，`lsmod`即可

### 3、卸载驱动模块

在任何目录下，`rmmod <module_name>`即可，注意其中`”module_name”`是`lsmod`显示的模块名称，而不是对应的`ko`文件名。