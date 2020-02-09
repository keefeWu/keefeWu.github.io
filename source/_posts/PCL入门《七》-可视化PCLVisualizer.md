---
title: PCL入门《七》-可视化PCLVisualizer
date: 2020-02-08 15:18:53
tags:
---
PCL的点云可视化的一种最常用的方法就是PCLVisualizer

它是基于VTK的可视化类，需要引入头文件

    
    
    #include <pcl/visualization/pcl_visualizer.h>

通常用法是

    
    
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addPointCloud (cloud, "cloud");
    viewer->spin();

这就构成了最基本的显示流程了，其中第一行创建一个窗口，窗口名字叫做3D
Viewer，这个窗口是个智能指针创建的，所以在它的生命周期结束前这个窗口都不会销毁。（实际上根据我的观察似乎程序结束前它都不会被销毁。。。）

然后使用addPointCloud
这个函数来添加点云，注意是添加到viewer这个对象里，你还可以在添加完这一组后再添加另一组，那么这个viewer里就会同时有两组，如果想删除掉某一组，就需要留意后面那个参数里，那个参数就是cloud的名字，你可以通过

    
    
    viewer->removePointCloud("cloud);

来移除cloud这个组里的所有点，但是保留了其他组的点。当然，如果添加了太多组，也可以使用

    
    
    viewer->removeAllPointClouds()

来移除所有的点。

  

如果想要添加座标系，可以使用

    
    
    viewer->addCoordinateSystem();

这个函数来添加座标系。

