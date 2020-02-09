---
title: PCL入门《一》-点云的数据结构
date: 2020-02-08 15:20:13
tags:
---
PCL最基本的数据类型就是PointXYZ

这个代表的是一个黑白的点，这个类包含了xyz的坐标，当然也有更高级的PointXYZRGB，这个里面不仅有坐标，还有点的RGB颜色值。

如果我们要定义一连串点云一般用的是PointCloud，这个类里面有个point变量，这个变量实际上用的是STL里面的vector。

PCL的example里通常都是这样定义的：

    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> );

如果要访问某一个点，则需要：

    
    
    cloud->points[i].x

可以看到，points是个vector变量，所以points[i]就是单个的点，这里访问了他的x的值，同理可以访问y和z，如果是PointXYZRGB则还有rgb，如果想往cloud这个变量里面添加一个点的信息，则只需要定一个PointXYZ（或PointXYZRGB）的变量，然后通过vector的push_back，加入到points这个变量里面。

    
    
    pcl::PointXYZ point;
          point.x = 2.0f - y;
          point.y = y;
          point.z = z;
          cloud.points.push_back(point);

如果有两个坐标相同的点，则颜色信息以最后的一个为准。

如果想要显示你的点云信息，需要创建一个PCLVisualizer的对象

pcl::visualization::PCLVisualizer::Ptr viewer (new
pcl::visualization::PCLVisualizer ("Cloud"));

然后执行这个对象的

viewer->addPointCloud (cloud,  "cloud");

这样就可以显示了，但是很快就消失了，所以感觉并没有什么用，所以需要加上下面一句，暂停和控制，

viewer->spin();

这样就是一个完整的显示了，可以通过鼠标和键盘旋转和放大这个点云来观察。

