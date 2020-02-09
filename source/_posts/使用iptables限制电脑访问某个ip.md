---
title: 使用iptables限制电脑访问某个ip
date: 2020-02-08 15:17:51
tags:
---
最近有个单机软件老要提醒我更新，我就用iptables把它的ip给屏蔽了，操作入下。

sudo iptables -I INPUT -s 1.1.1.1 -j DROP  

使用上面这个命令就把1.1.1.1这个ip给屏蔽了，以后电脑再也上不了这个了。

使用如下命令可以查看屏蔽了什么：

sudo iptables -L -n -v  

 **删除已添加的iptables规则**

将所有iptables以序号标记显示，执行：

    
    
    iptables -L -n --line-numbers

比如要删除INPUT里序号为8的规则，执行：

    
    
    iptables -D INPUT 8

  

