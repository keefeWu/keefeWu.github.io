---
title: linux命令
date: 2020-02-08 15:21:21
tags:
---
ldd:查看某个库的依赖库。如ldd libxxx.so  
则会打印这个libxxx.so依赖的所有库，用于检查编译时候只添加了头文件而未添加依赖的动态库造成的undefined symbol错误。

ps aux：查看所有进程的id，方便杀死某个后台进程，通常连接  
kill (id) 可以通过关键字搜索 ps -aux |grep python  
搜索所有带python的进程

tail：将文本文件内容实时打印出来，通常用来打印日志，例如：  
tail -f log.out

cat /proc/sys/vm/swappiness : 查看启用交换分区时空闲内存还剩多少 默认是60，就是说内存占用率在40%的时候就开始启用交换分区

修改方法：  
sudo gedit /etc/sysctl.conf  
打开这个文件，在最后加上vm.swappiness=10  
就变成内存占用90%的时候才开启交换分区

如果只想临时生效可以执行  
sudo sysctl vm.swappiness=10

安装deb程序:

sudo apt install [name]  
例如： sudo apt install a.deb  
如果出现有些需要更新，则执行  
sudo apt –fix-broken install

