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

- 暂存文件的命令：**git add <文件名>**
- 放弃未暂存文件的修改命令：**git checkout – <文件名>**
- 将被修改的文件暂存并提交的命令：**git commit -a**

M - 被修改，A - 被添加，D - 被删除，R - 重命名，?? - 未被跟踪 等等

### git diff

- 尚未缓存的改动：**git diff**
- 查看已缓存的改动： **git diff --cached**
- 查看已缓存的与未缓存的所有改动：**git diff HEAD**
- 显示摘要而非整个 diff：**git diff --stat**

### git commit

​	git commit 将缓存区内容添加到仓库中，可以在后面加-m选项，以在命令行中提供提交注释，格式如下：

```bash
git commit -m "第一次版本提交"
git commit -am "第一次版本提交" //跳过add这一步
```

### git rm

```bash
//简单地从工作目录中手工删除文件
git rm <file>

//如果删除之前修改过并且已经放到暂存区域的话，则必须要用强制删除选项 -f
git rm -f <file>

//递归删除，即如果后面跟的是一个目录做为参数，则会递归删除整个目录中的所有子目录和文件：
git rm –r *
```

### git mv

​	git mv 命令用于移动或重命名一个文件、目录、软连接，如要将一个test.txt文件重命名为newtest.txt，则可以使用如下命令：

```bash
git mv test.txt newtest.txt
```

## Git 分支管理

- **git branch**：查看分支命令
- **git branch (branchname)**：创建分支命令
- **git checkout (branchname)**：切换分支命令
- **git merge**：合并分支命令
- **git branch -d (branchname)**：删除分支命令

## Git 提交历史

**git log**

- –-oneline ：查看历史记录的简洁版本
- –-graph ：查看历史中什么时候出现了分支、合并
- –-reverse ：逆向显示所有日志
- –-author ：查找指定用户的提交日志
- –-since、–-before、 --until、–-after： 指定筛选日期
- –-no-merges ：选项以隐藏合并提交

## Git 远程仓库

- **git remote add**：添加远程仓库
- **git remote**：查看当前的远程仓库
- **git fetch**、**git pull**：提取远程仓仓库
- **git push**：推送到远程仓库
- **git remote rm**：删除远程仓库

```bash
git remote add [alias] [url]
git push [alias] [branch]
```

