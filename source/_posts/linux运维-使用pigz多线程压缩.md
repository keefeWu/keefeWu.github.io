---
title: linux运维-使用pigz多线程压缩
date: 2020-01-21 09:37:33
tags: [运维]
categories: 
- 运维
---
因为tar zip是单线程的压缩，压缩起来很慢，这个使用使用pigz工具辅助就会使用多线程了。
安装

sudo apt install pigz
压缩

tar cvf - test.txt | pigz > test.tar.gz
解压到指定目录

转自： http://unix.stackexchange.com/questions/198958/unpigz-and-untar-to-a-specific-directory

I found three solutions:
With GNU tar, using the awesome -I
 option:
tar -I pigz -xvf /path/to/archive.tar.gz -C /where/to/unpack/it/

With a lot of Linux piping (for those who prefer a more geeky look):
unpigz < /path/to/archive.tar.gz | tar -xvC /where/to/unpack/it/

More portable (to other tar
 implementations):
unpigz < /path/to/archive.tar.gz | (cd /where/to/unpack/it/ && tar xvf -)

(You can also replace tar xvf -
 with pax -r
 to make it [POSIX](https://en.wikipedia.org/wiki/POSIX)-compliant, though not necessarily more portable on Linux-based systems).

Credits go to [@PSkocik](http://unix.stackexchange.com/users/23692/pskocik) for a proper direction, [@Stéphane Chazelas](http://unix.stackexchange.com/users/22565/st%C3%A9phane-chazelas) for the 3rd variant and to the author of [this](http://stackoverflow.com/a/29270282/2202101) answer.

[Click and drag to move]
使用tar+pigz+ssh实现大数据的高效传输, 流式压缩传输

http://www.cnblogs.com/chjbbs/p/6472236.html
磁盘读取---->打包---->压缩------>传输---->解压缩-->拆包---->落盘
|->tar |->gzip |->ssh |->gzip |->tar
tar -c test/ |pigz |ssh -c arcfour128 目标IP "gzip -d|tar -xC /data" # 解压
tar -c test/ |pigz |ssh -c arcfour128 目标IP "cat >/data/test.tar.gz" # 不解压

[Click and drag to move]



作者：幻视快银
链接：https://www.jianshu.com/p/7d956a21f0ab
來源：简书
简书著作权归作者所有，任何形式的转载都请联系作者获得授权并注明出处。