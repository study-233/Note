# Linux cp 命令

## **选项说明**：

- `-r` 或 `-R`：递归复制目录及其内容（用于复制目录）。
- `-i`：交互模式，覆盖前提示用户确认。
- `-f`：强制复制，覆盖目标文件而不提示。
- `-v`：显示详细的复制过程（verbose）。
- `-p`：保留文件的原始属性（如权限、时间戳等）。
- `-a`：归档模式，等同于 `-dpR`，保留所有文件属性和递归复制目录。
- `-u`：仅当源文件比目标文件新时才复制（更新模式）。
- `-l`：创建硬链接而不是复制文件。
- `-s`：创建符号链接（软链接）而不是复制文件。

## 实例

**1. 复制文件到目标目录**

```
cp file.txt /path/to/destination/
```

将 file.txt 复制到 /path/to/destination/ 目录中。

**2. 复制文件并重命名**

```
cp file.txt /path/to/destination/newfile.txt
```

将 file.txt 复制到 /path/to/destination/ 目录并重命名为 newfile.txt。

**3. 递归复制目录**

```
cp -r /path/to/source_dir /path/to/destination/
```

将 source_dir 目录及其内容递归复制到 destination 目录。

**4. 交互模式复制**

```
cp -i file.txt /path/to/destination/
```

如果目标位置已存在同名文件，会提示用户确认是否覆盖。

**5. 保留文件属性**

```
cp -p file.txt /path/to/destination/
```

复制文件并保留其原始属性（如权限、时间戳等）。

**6. 仅复制更新的文件**

```
cp -u file.txt /path/to/destination/
```

仅当 file.txt 比目标文件新时才复制。

**7. 显示复制过程**

```
cp -v file.txt /path/to/destination/
```

显示复制的详细信息。

**8. 创建硬链接或符号链接**

```
cp -l file.txt /path/to/destination/  # 创建硬链接
cp -s file.txt /path/to/destination/  # 创建符号链接
```

**9. 复制多个文件到目录**

```
cp file1.txt file2.txt /path/to/destination/
```

将多个文件复制到目标目录。

**10. 使用通配符复制**

```
cp *.txt /path/to/destination/
```

复制所有 .txt 文件到目标目录。

**11. 结合 find 命令复制特定文件**

```
find /path/to/source -name "*.log" -exec cp {} /path/to/destination/ \;
```

## **Linux 将一个文件夹的所有内容拷贝到另外一个文件夹**

cp 命令使用 **-r** 参数可以将 packageA 下的所有文件拷贝到 packageB 中：

```
cp -r /home/packageA/* /home/cp/packageB/
```

将一个文件夹复制到另一个文件夹下，以下实例 packageA 文件会拷贝到 packageB 中：

```
cp -r /home/packageA /home/packageB
```

运行命令之后 packageB 文件夹下就有 packageA 文件夹了。