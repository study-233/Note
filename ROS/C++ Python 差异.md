# C++

1. 在 src 中创建cpp文件
2. 在 CMakeList 中添加

```cmake
add_executable(chao_node src/chao_node.cpp)
target_link_libraries(chao_node
  ${catkin_LIBRARIES}
)
```



# Python

1. 用python编写只需在创建完软件包后编译一次即可

2. 需要添加权限 chmod +x 文件名 (x是eXecute缩写，添加可执行权限)

