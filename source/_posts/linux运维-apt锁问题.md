---
title: linux运维-apt锁问题
date: 2020-01-07 18:59:29
tags: [运维, apt]
categories: 
- 运维
---
　　在想要更新资源的时候出现了以下问题：
　　而出现此问题的原因是因为可能是有另外一个程序正在运行，导致资源被锁不可用。而导致资源被锁的原因，可能是上次安装时没正常完成，而导致出现此状况。
## 解决办法：
### 在终端敲入这两句命令：
```
sudo rm /var/cache/apt/archives/lock
sudo rm /var/lib/dpkg/lock
```

再继续安装就好了。

作者：为理想而奋斗的啊航 
来源：CSDN 
原文：https://blog.csdn.net/weixin_42489042/article/details/81296472 
版权声明：本文为博主原创文章，转载请附上博文链接！
