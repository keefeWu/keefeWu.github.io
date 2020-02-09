---
title: git常用命令《按流程列举》
date: 2020-02-08 15:19:13
tags:
---
常规提交流程：

    
    
    git add readme.txt
    git commit -m "append GPL"
    git push

add就是添加文件，你可以添加多个文件或者文件夹，只要用空格隔开就可以了  
commit -m 是提交，引号内的参数是这个版本的说明。  
push 是上传到服务器，如果你没有服务器或者没有网络也可以暂时不执行这一步骤

版本回退：  
1、回退到上一个版本：

    
    
    git reset --hard HEAD^

其中head^代表向上一个指针，head^^代表向上两个指针，依次类推。  
当然，也可以查出版本号来回退

    
    
    git log
    git reset --hard 3628164

通过log查出版本号，当然也可以用其他方法，例如带有管理系统的git服务器，github ,gitlab等等  
然后通过rest –hard回退

也可以使用checkout

    
    
    git checkout 3628164

然后跟着他的提示push，后续更新

2、  
删除文件

    
    
    git rm --cached a.cpp a.h
    git commit -m 'delete files'
    git push 

3、移动文件

    
    
    git mv a.cpp b.cpp
    git commit -m 'move a.cpp to b.cpp'
    git push 

