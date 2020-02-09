---
title: opencv中Rect类的神奇用法
date: 2020-02-08 15:21:17
tags:
---
转载自<http://blog.csdn.net/q6324266/article/details/52403744>

最近发现[OpenCV](http://lib.csdn.net/base/opencv
"OpenCV知识库")中的Rect类非常神奇，其中很多函数使用起来极其方便。一下列举一些比较实用的函数：

  1. size()函数返回矩形的尺寸大小。返回类型为cv::Size。
  2. area()函数返回矩形的面积，也就是矩形包含的像素点个数。也就是矩形的（宽*高）的值。  

  3. contains(Point)能检测点是否在矩形内。  

  4. inside（Rect）检测矩形是否在矩形内。  

  5. tl()返回矩形左上角的点坐标。即top-left。
  6. br()返回矩形右下角点坐标。即bottom-right。

  

还有更神奇的招数！如果要求两个矩形的交集与并集，opencv的Rect类提供了非常方便的方式。

**[cpp]** [view plain](http://blog.csdn.net/q6324266/article/details/52403744#
"view plain") [copy](http://blog.csdn.net/q6324266/article/details/52403744#
"copy")

  1. Rect rect = rect1 & rect2; 
  2. Rect rect = rect1 | rect2; 

  
如果想将Rect平移，可以这样写：

**[cpp]** [view plain](http://blog.csdn.net/q6324266/article/details/52403744#
"view plain") [copy](http://blog.csdn.net/q6324266/article/details/52403744#
"copy")

  1. Rect r1(0, 0, 5, 5); 
  2. Point p(2, 3); 
  3. Rect r2 = r1 + p;<span style="white-space:pre"> </span>//平移

  
如果想改变矩形的尺寸大小，可以这样写：

**[cpp]** [view plain](http://blog.csdn.net/q6324266/article/details/52403744#
"view plain") [copy](http://blog.csdn.net/q6324266/article/details/52403744#
"copy")

  1. Rect r1(0, 0, 5, 5); 
  2. Size s(-1, -1); 
  3. Rect r2 = r1 + s;<span style="white-space:pre"> </span>//改变尺寸大小

  
但是据我观察，rect裁出来的数据只是加上了坐标限制，但是data里存的数据还是全部的数据，所以如果需要访问data的话还是要单点拷贝了，就是写一个循环一个一个赋值。

