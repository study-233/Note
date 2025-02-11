# 修复缺少依赖关系 

`sudo apt --fix-broken install`

# Linux中的“~”、“/”、“./”分别代表什么

- “~” ：表示主目录，也就是当前登录用户的用户目录
- “/” ：是指[根目录](https://so.csdn.net/so/search?q=根目录&spm=1001.2101.3001.7020)：就是所有目录最顶层的目录
- “./” ：表示当前目录，./ 一般需要和其他文件夹或者文件结合使用，指代当前目录下的东西
- “. .” ：表示上级目录

```bash
cd ~  	--> 	/home/andy

cd /    --> 	/
```

# png --> jpg

保留源文件

```
for file in *.png; do convert "$file" "${file%.png}.jpg"; done
```

不保留

```
for file in *.png; do
    convert "$file" "${file%.png}.jpg" && rm "$file"
done
```

# 统计文件个数

tree

ls | wc -l
