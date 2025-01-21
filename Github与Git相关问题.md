# GitHub 与Git 相关问题 
## 问题01
### Q
    已开启代理但是git bash中还是显示
    Failed to connect to github.com port 443 after 21102 ms: Could not connect to server
### A
```bash
配置 Git 使用代理
确保 Git 使用与系统代理设置相同的端口。可以通过以下命令配置 Git 的代理：
git config --global http.proxy http://127.0.0.1:7890
git config --global https.proxy http://127.0.0.1:7890
```

## 问题02

### Q

```
ssh: connect to host github.com port 22: Connection timed out
fatal: 无法读取远程仓库
```

### A

如果22号端口不行，那就换一个端口

操作方法：

1. 进入~/.ssh下
   `cd ~/.ssh`

2. 创建一个config文件(这里我用的vim编辑器)
   `vim config`
   编辑文件内容：

```
Host github.com
User git
Hostname ssh.github.com
PreferredAuthentications publickey
IdentityFile ~/.ssh/id_rsa
Port 443

Host gitlab.com
Hostname altssh.gitlab.com
User git
Port 443
PreferredAuthentications publickey
IdentityFile ~/.ssh/id_rsa
```

3. 保存退出
   检查是否成功
   `ssh -T git@github.com`

