# git常用指令

## 创建仓库

### git init 

​	Git使用git init命令来初始化一个Git仓库，执行完git init命令后，会生成一个.git目录，该目录包含了资源数据，且只会在仓库的根目录生成。

```bash
git init
```

### git clone 

​	使用git clone命令可以从Git仓库拷贝项目，类似于SVN中的 svn checkout，命令格式为：

```bash
git clone <url> [directory]
git clone git://github.com/schacon/grit.git newgit
```

## 基本指令

### git config

```bash
//配置Git
git config --global user.name '你的用户名'
git config --global user.email '你的邮箱'

//列出当前的 Git 配置信息
git config --global -l 
```

### git add

​	git add 命令可将文件添加到缓存，如新项目中，添加所有文件很普遍，可以使用如下命令：

```bash
git add . //添加所有文件
git add *.txt //添加文本文件
```

### git status

​	使用 git status 命令来查看相关文件的状态

```bash
git status
```

