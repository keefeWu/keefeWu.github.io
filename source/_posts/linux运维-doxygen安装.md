---
title: linux运维-doxygen安装
date: 2020-02-08 15:11:16
tags:
---
doxygen是一个非常好的文档工具，至少能够规范注释。  
安装也非常容易，从github上下载下来，用cmake make install就可以了。  
使用方法需要注意一下，进入项目目录，用

    
    
    doxygen -g -s Doxygen
    

来生成一个Doxygen的配置文件，当然名字可以自定义，`-g`应该是generate的意思，`-s`的意思应该是short之类的，不需要注释，他们的注释真的超级长，强烈建议加上这个。  
然后用文本编辑器编辑这个配置文件，重点是这几项

    
    
    OUTPUT_DIRECTORY       = doc/
    INPUT                  = src/
    RECURSIVE              = YES
    

`OUTPUT_DIRECTORY`是文档生成目录  
`INPUT`是源代码目录  
`RECURSIVE`是文件递归，这里略坑，默认是no，如果不开的话只会在最表层找代码，如果你的代码都在目录里面将会什么都找不到。  
然后去doc这个目录里面找那个html文件双击打开就可以了。

