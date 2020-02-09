---
title: PCL入门《九》-输入与输出
date: 2020-02-08 15:18:03
tags:
---
今天讲解一下PCL的输入输出。

首先是输入，从pcd文件中读取点云

    
    
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile<PointT>(pcd_file, *cloud) == -1)
        {
            std::cout << "Cloud reading failed." << std::endl;
            return (-1);
        }

其中输入就是文件路径，还有一个点云的对象，记住不是指针，所以要用个*去找到他的引用。

头文件

    
    
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl/io/pcd_io.h>

接下来是输出，把点云写入到pcd文件

    
    
        printf("save data in ./data/cloud\n");
        cloud->width=cloud->size();
        cloud->height=1;
        pcl::io::savePCDFileASCII ("./data/cloud.pcd", *cloud);
    

是首先确定尺寸要一致，就是width和height的乘积要保持和点的个数一致，然后传入路径和点云的引用就可以了。

