---
title: 向caffe的docker中添加opencv并保存
date: 2020-01-07 19:17:53
tags: [caffe, docker, opencv]
categories: 
- caffe
---

参考自：http://www.docker.org.cn/book/docker/docer-save-changes-10.html

首先学会安装opencv，因为caffe的docker是没有源的，所以需要手动添加进去
```
echo "deb http://de.archive.ubuntu.com/ubuntu precise main restricted universe" | sudo tee -a /etc/apt/sources.list
echo "deb-src http://de.archive.ubuntu.com/ubuntu precise restricted main multiverse universe" | sudo tee -a /etc/apt/sources.list
echo "deb http://de.archive.ubuntu.com/ubuntu precise-updates main restricted universe" | sudo tee -a /etc/apt/sources.list
echo "deb-src http://de.archive.ubuntu.com/ubuntu precise-updates restricted main multiverse universe" | sudo tee -a /etc/apt/sources.list
```

添加上之后更新源
```
apt update
```
然后安装python-opencv
```
apt install python-opencv
```
保存对容器的修改

当你对某一个容器做了修改之后（通过在容器中运行某一个命令），可以把对容器的修改保存下来，这样下次可以从保存后的最新状态运行该容器。docker中保存状态的过程称之为committing，它保存的新旧状态之间的区别，从而产生一个新的版本。

目标：

首先使用docker ps -l命令获得安装完ping命令之后容器的id。然后把这个镜像保存为caffe_opencv。


我就不列举自己的了，抄了一个例子

提示：

1. 运行docker commit，可以查看该命令的参数列表。

2. 你需要指定要提交保存容器的ID。(译者按：通过docker ps -l 命令获得)

3. 无需拷贝完整的id，通常来讲最开始的三至四个字母即可区分。（译者按：非常类似git里面的版本号)

正确的命令：
```
docker commit 698 learn/ping
```

执行完docker commit命令之后，会返回新版本镜像的id号。



