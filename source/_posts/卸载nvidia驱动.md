---
title: 卸载nvidia驱动
date: 2020-01-09 09:18:16
tags: [运维, nvidia, cuda]
categories: 
- 运维
---

# 卸载nvidia驱动
卸载NV驱动和安装一样，首先ctrl+Alt+F2进入命令行状态，然后停止lightdm
```
sudo service lightdm stop
```
或者
```
sudo /etc/init.d/lightdm stop
```
卸载命令位置
<code>/usr/bin/nvidia-uninstall</code>，以下命令即可卸载。
```
sudo /usr/bin/nvidia-uninstall
```
不找这个命令的位置，也可以
```
sudo apt-get install autoremove --purge nvidia*
```
# 卸载cuda
cuda的默认安装在 /usr/local/cuda-8.0下，用下面的命令卸载：
```
sudo /usr/local/cuda-8.0/bin/uninstall_cuda-8.0.pl
```
会出现长时间的刷屏现象，说明cuda文件其实还蛮多的~~ 