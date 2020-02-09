---
title: PCL入门《三》平面的法线
date: 2020-02-08 15:20:02
tags:
---
通常我们需要估计平面的方向,这就需要用到法线了，法线就是指垂直平面的线。

PCL中有自动求出法线的方法，我们来看一看，

首先假设我们有一个平面的点云cloud_xyz

要求出法线首先我们先定义一个法线估计的对象ne

pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

然后把要求的平面点云传给他

ne.setInputCloud (cloud_xyz);

这个方法求出cloud_xyz上每个点的法线方向，他是根据每个点附近的点云组成的平面求出的，所以我们需要设置一个半径，用来估计附近多大的范围取点来算平面，

ne.setRadiusSearch (0.05);

最后定义一个normal的对象来接受计算出来的法线

pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new
pcl::PointCloud<pcl::Normal>);

计算并把结果保存在cloud_normals里

ne.compute (*cloud_normals);

这时算出来的法线就在这个对象里了，他里面的元素是和点云一一对应的，我们要访问每个点的法线方向，可以这样

for(int ix=0;ix<cloud_normals->points.size();ix++)

{

if(isnan(cloud_normals->points[ix].normal_x)||

isnan(cloud_normals->points[ix].normal_y)||

isnan(cloud_normals->points[ix].normal_z))

{

continue;

}

}

这里的isnan是判断法线是否为nan，因为如果一个点周围找不到足够的点计算平面，则法线中会赋值为nan，如果需要直接当作数字来用的同学们一定要注意这一点，使用我这种方法来判断。normal_x就是法线的x方向了，y
z同理。然后PCL算出的法线是不知道正方向的，所以这点需要人为去判断正负。

