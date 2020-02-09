---
title: python常用函数
date: 2020-02-08 15:21:06
tags:
---
find : 查找字符串所在位置，常用来做截取。

name='a_b_c'

name.find('_')=1

可以设置从第几个字符开始，例如

name.find('_',2)=3

可以看到跳过了第一个_，但是计数方法还是从头开始。

  

lambda: 匿名函数，快速定义一个函数，无序繁琐的固定结构，可以直接在代码中使用。

    
    
    fun_a=lambda x: x+1
    
    
    print fun_a(5)

  
输出的结果就是6

