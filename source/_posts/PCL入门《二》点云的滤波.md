---
title: PCL入门《二》点云的滤波
date: 2020-02-08 15:20:05
tags:
---
PCL官方给的filter示例实际上是一个空间切割，也就是只保留设定范围内的点，超出边界的就过滤掉了。直接看例子：

    
    
    pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 1.0);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloud_filtered);

这里cloud就是需要过滤的点云，cloud_filtered就是过滤后的点云，使用setFilterFieldName来设置过滤Z轴，用setFilterLimits来设置范围，也就是只保留z坐标在0-1之间的点，其他的全部不要了。

  

另外一种滤波是圆形滤波，这个是遍历图像中每一个点，如果他周围给定半径内点的数量小于自己设置的一个阈值，那么这一个点将被忽略。

    
    
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setRadiusSearch (0.2);
    pass.setMinNeighborsInRadius (20);
    pass.filter (*cloud_filtered);
    
    

  
依然是创建一个PassThrough的滤波对象，然后对cloud点云进行滤波，设置半径为0.2，设置周边点的阈值为20，如果每个点0.2范围内的点的数量少于20则忽略这个点。

  

注意，注释掉的那一行

    
    
    pass.setFilterLimitsNegative (true);

  

代表了是否显示被过滤掉的点，如果设置为true则将被过滤的以红色表示，没过滤的用绿色表示。默认是false.

