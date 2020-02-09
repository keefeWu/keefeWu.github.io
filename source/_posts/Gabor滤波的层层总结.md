---
title: Gabor滤波的层层总结
date: 2020-02-08 15:21:44
tags:
---
<http://blog.csdn.net/u011021773/article/details/54232169>  

声明：这篇博客是我学习gabor滤波的笔记，根据进度层层总结，在完全学会前会看起来很混乱，但是我保证在终止连载以后一定马上整理成Gabor的介绍博客。

我学习的起因是阅读了《Human Age Estimation Using Bio-inspired Features》这篇文章。

核的大小：

文章里面出现了这样一个表格：

![](0.png)  

这个表格的右半部份是gabor滤波的参数，其中含义是当周期T为2PI的时候，sigma与lamda做自变量得出来的size为表中所示。

如：第一行sigma=2.0,lamda=2.5，则可算出size是5*5.

