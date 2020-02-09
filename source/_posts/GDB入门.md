---
title: GDB入门
date: 2020-02-08 15:18:39
tags:
---
首先在CMAKE里面添加编译选项，更改成DEBUG版本

set(CMAKE_BUILD_TYPE "Debug")  

编译后用gdb运行程序，例如：

gdb test

如果显示load成功，就可以设置参数了

set args 1.0 2.0 1.2

这里假设的是三个数字参数

设置完之后可以设置断点，使用break 100

就是在第100行设置断点

然后使用r去执行

当运行到断点时，需要打印某个变量，例如i，那么就用

p i

p是print的简写

然后按n就是分步执行 next的简写

c就是继续执行 continue的简写

如果想指定文件设置断点使用

break test.cpp:20

就代表是test.cpp 这个文件的第20行设置断点

如果想调试段错误，使用

up和down可以灵活切换内外层

