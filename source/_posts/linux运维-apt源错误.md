---
title: linux运维-apt源错误
date: 2020-01-07 19:21:22
tags: [运维, apt]
categories: 
- 运维
---
解决E: Unmet dependencies. Try 'apt-get -f install' with no packages (or specify a solution).的一种方法

最近准备装一个软件，更新了apt源却失效了，导致没装成功，其他的软件也装不了了。

解决办法：

手动去源列表里面删除刚刚添加的那一项

在/etc/apt/sources.list地方

然后

sudo apt-get -f install

就恢复正常了
