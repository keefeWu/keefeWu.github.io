---
title: Ubuntu蓝屏进不了图形界面的解决办法
date: 2020-02-08 15:19:47
tags:
---
Ubuntu有时强制关机会损坏图形界面，但是不用着急，可以通过命令行自动重装桌面。方法如下：

Ctrl+Alt+F4进入命令行模式

依次执行：

sudo dpkg --configure -a

sudo apt-get install xserver-xorg-lts-utopic sudo dpkg-repack xserver-xorg-
lts-utopic

（有些系统的名字可能不一样，但是前缀都是一样的，可以先输入前缀按tab键提示找到最像的）

sudo reboot

  

重起之后熟悉的桌面又回来了  

