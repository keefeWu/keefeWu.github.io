---
title: PCL入门《六》使用Region-growing进行平面分割
date: 2020-02-08 15:19:01
tags:
---
很多时候我们需要分割平面，检测每个平面，这个时候可以使用PCL自带的区域增长方法。  
  
  

话不多说直接上代码：

    
    
      pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
      pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
      normal_estimator.setSearchMethod (tree);
      normal_estimator.setInputCloud (cloud);
      normal_estimator.setKSearch (50);
      normal_estimator.compute (*normals);
    
      pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
      reg.setMinClusterSize (50);
      reg.setMaxClusterSize (1000000);
      reg.setSearchMethod (tree);
      reg.setNumberOfNeighbours (30);
      reg.setInputCloud (cloud);
      //reg.setIndices (indices);
      reg.setInputNormals (normals);
      reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
      reg.setCurvatureThreshold (1.0);
    
      std::vector <pcl::PointIndices> clusters;
      reg.extract (clusters);
    

因为这个算法是根据法线来计算的，所以我先算出了法线，在

    
    
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;

以上的代码是求法线的部分，那么我们从这行开始往下讲，就都是区域增长的内容了。  
  
  
首先还是先建立了一个区域增长的对象reg  
  
  
然后设置平面包含的最少点数（这个参数非常重要，小于这个参数的平面会被忽略不计）  
  
  
然后设置最大的点数，原理同上，但是一般我们希望的是无穷大，所以可以设大一点，当然如果你有特殊要求可以按自己需求来  
  
  
然后设置搜索方法，之前我们声明了使用kd树的方法，所以直接用就可以了，这也是默认的方法。  
  
  
然后设置参考的邻域点数，也就是看看周边的多少个点来决定这是一个平面（这个参数至关重要，决定了你的容错率，如果设置的很大，那么从全局角度看某一个点稍微有点歪也可以接受，如果设置的很小则通常检测到的平面都会很小）  
  
  
然后输入要检测的点云cloud  
  
  
然后输入点云的法线（计算法线的方法参考之前的博客，这里给个传送门
http://blog.csdn.net/u011021773/article/details/78247657）  
  
  
然后设置判断的阈值，大概也就是两个法线在多大的夹角内还可以当做是共面的。  
  
  
最后也是一个弯曲的阈值，这个决定了比当前考察的点和平均的法线角度，决定是否还有继续探索下去的必要。（也就是假设每个点都是平稳弯曲的，那么normal的夹角都很小，但是时间长了偏移的就大了，这个参数就是限制这个用的）  
  
  
然后就可以把结果输出到一个簇里面，这个簇会自动把每个平面分成一个vector，可以打印下来看看  

    
    
    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;

也可以把检测到的每个平面涂上不同的颜色

    
    
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
      pcl::visualization::CloudViewer viewer ("Cluster viewer");
      viewer.showCloud(colored_cloud);
      while (!viewer.wasStopped ())
      {
      }

效果如图  

![](0.png)

当然，实际效果可能没有这么好，如果有这么多平面需要分割我建议先手工把它减少一点，这个算法对单独检测一两个特定规格的平面还是很准的，但是这么多的真心不建议。  
  
  
这个算法是基于法线计算的，输入的是无颜色的点云。还有另一个方法是可以基于颜色计算的，我们以后会继续讲解。  

