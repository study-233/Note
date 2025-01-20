# GitHub 连接问题 
## 问题01
### Q
    已开启代理但是git bash中还是显示
    Failed to connect to github.com port 443 after 21102 ms: Could not connect to server
### A
    配置 Git 使用代理
    确保 Git 使用与系统代理设置相同的端口。可以通过以下命令配置 Git 的代理：
    git config --global http.proxy http://127.0.0.1:7890
    git config --global https.proxy http://127.0.0.1:7890
