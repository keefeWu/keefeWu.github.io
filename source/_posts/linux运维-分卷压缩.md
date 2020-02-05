---
title: linux运维-分卷压缩
date: 2020-02-05 10:35:47
tags: [运维]
categories: 
- 运维
---
转载自 
[https://blog.csdn.net/heroacool/article/details/73881680linux](https://blog.csdn.net/heroacool/article/details/73881680linux) 

拷贝4G以上的文件到U盘经常会出现文件过大，拷贝失败，所以经常需要分卷压缩。操作如下1.分卷压缩tar cvzpf - test | split -d -b 500m1上面的命令是将test这个文件夹分卷压缩，每卷500m，注意test前面有空格。压缩完之后，会出现很多名称为x00 x01 x02 …的文件，每个文件的大小均为500M，最后一个分卷可能会小一些。2.解压 首先合并所有的分卷压缩文件：cat x* > test.tar.gz1然后解压：tar xzvf test.tar.gz