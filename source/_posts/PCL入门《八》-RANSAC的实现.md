---
title: PCL入门《八》-RANSAC的实现
date: 2020-02-08 15:18:18
tags:
---
之前发国一篇文章讲解了ransac的基本原理，PCL也有相应的代码的实现，今天来讲以下如何使用。

先看PCL官网的示例：

    
    
    #include <iostream>
    #include <pcl/ModelCoefficients.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/point_types.h>
    #include <pcl/sample_consensus/method_types.h>
    #include <pcl/sample_consensus/model_types.h>
    #include <pcl/segmentation/sac_segmentation.h>
    #include <pcl/filters/extract_indices.h>

  

    
    
    int
     main (int argc, char** argv)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
      // Fill in the cloud data
      cloud->width  = 15;
      cloud->height = 1;
      cloud->points.resize (cloud->width * cloud->height);
    
      // Generate the data
      for (size_t i = 0; i < cloud->points.size (); ++i)
      {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1.0;
      }
    
      // Set a few outliers
      cloud->points[0].z = 2.0;
      cloud->points[3].z = -2.0;
      cloud->points[6].z = 4.0;
    
      std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
      for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
                            << cloud->points[i].y << " "
                            << cloud->points[i].z << std::endl;
    
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);
    
      seg.setInputCloud (cloud);
      seg.segment (*inliers, *coefficients);
    
      if (inliers->indices.size () == 0)
      {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
      }
    
      std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                          << coefficients->values[1] << " "
                                          << coefficients->values[2] << " " 
                                          << coefficients->values[3] << std::endl;
    
      std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
      for (size_t i = 0; i < inliers->indices.size (); ++i)
        std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                   << cloud->points[inliers->indices[i]].y << " "
                                                   << cloud->points[inliers->indices[i]].z << std::endl;
    
      return (0);
    }

其中要注意的变量有三个，

输入点云：cloud

输出的index: indices
这个变量表示它计算完以后把所有符合条件的点的id返回出来，例如这里使用的模型是SACMODEL_PLANE也就是寻找一个平面，当然PCL还有其他各种不同的参数，可以找到不同的物体，一会我会列举出来

平面的法线信息，最后打印的values0，1，2分别是法线的xyz

PCL能找的物体种类，只需要改变setModelType就可以了

  * [SACMODEL_PLANE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_plane.html)\- used to determine plane models. The four coefficients of the plane are its [Hessian Normal form](http://mathworld.wolfram.com/HessianNormalForm.html): [normal_x normal_y normal_z d]
  * [SACMODEL_LINE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_line.html)\- used to determine line models. The six coefficients of the line are given by a point on the line and the direction of the line as: [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z]
  * [SACMODEL_CIRCLE2D ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_circle2_d.html)\- used to determine 2D circles in a plane. The circle's three coefficients are given by its center and radius as: [center.x center.y radius]
  * [SACMODEL_CIRCLE3D ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_circle3_d.html)\- used to determine 3D circles in a plane. The circle's seven coefficients are given by its center, radius and normal as: [center.x, center.y, center.z, radius, normal.x, normal.y, normal.z]
  * [SACMODEL_SPHERE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_sphere.html)\- used to determine sphere models. The four coefficients of the sphere are given by its 3D center and radius as: [center.x center.y center.z radius]
  * [SACMODEL_CYLINDER ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_cylinder.html)\- used to determine cylinder models. The seven coefficients of the cylinder are given by a point on its axis, the axis direction, and a radius, as: [point_on_axis.x point_on_axis.y point_on_axis.z axis_direction.x axis_direction.y axis_direction.z radius]
  * [SACMODEL_CONE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_cone.html)\- used to determine cone models. The seven coefficients of the cone are given by a point of its apex, the axis direction and the opening angle, as: [apex.x, apex.y, apex.z, axis_direction.x, axis_direction.y, axis_direction.z, opening_angle]
  * SACMODEL_TORUS - not implemented yet
  * [SACMODEL_PARALLEL_LINE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_parallel_line.html)\- a model for determining a line parallel with a given axis, within a maximum specified angular deviation. The line coefficients are similar to [SACMODEL_LINE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_line.html).
  * [SACMODEL_PERPENDICULAR_PLANE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_perpendicular_plane.html)\- a model for determining a plane perpendicular to an user-specified axis, within a maximum specified angular deviation. The plane coefficients are similar to [SACMODEL_PLANE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_plane.html).
  * SACMODEL_PARALLEL_LINES - not implemented yet
  * [SACMODEL_NORMAL_PLANE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_normal_plane.html)\- a model for determining plane models using an additional constraint: the surface normals at each inlier point has to be parallel to the surface normal of the output plane, within a maximum specified angular deviation. The plane coefficients are similar to [SACMODEL_PLANE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_plane.html).
  * [SACMODEL_PARALLEL_PLANE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_parallel_plane.html)\- a model for determining a plane parallel to an user-specified axis, within a maximum specified angular deviation. [SACMODEL_PLANE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_plane.html).
  * [SACMODEL_NORMAL_PARALLEL_PLANE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_normal_parallel_plane.html)defines a model for 3D plane segmentation using additional surface normal constraints. The plane must lie parallel to a user-specified axis. SACMODEL_NORMAL_PARALLEL_PLANE therefore is equivalent to SACMODEL_NORMAL_PLANE + SACMODEL_PARALLEL_PLANE. The plane coefficients are similar to [SACMODEL_PLANE ](http://docs.pointclouds.org/1.8.1/classpcl_1_1_sample_consensus_model_plane.html).

这个例子程序只执行了一次RANSAC，如果想让它找出所有的平面怎么办呢，就使用PCL提供的extrac方法，把找到的index删掉，然后再剩余的里面再找，直到剩下的点很少了为止。

    
    
    while (cloud->points.size() > min_points_threshold * total_points)
        {
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            do_ransac(cloud, inliers, coefficients);
            ///get inliers and outliers
            pcl::PointCloud<PointT>::Ptr outliers_cloud, inliers_cloud;
            inliers_cloud = filter_cloud(cloud, inliers, false);
            outliers_cloud = filter_cloud(cloud, inliers, true);
            
            if (density < 0.8 || inliers_cloud->points.size() < 50) //TODO: turn this into tunable parameters
            {
                cloud = outliers_cloud;
                continue;
            }
            if (density > biggest_density)
            {
                biggest_density = density;
                biggest_density_id = plane_coefs.size();
            }
    
            plane_coefs.push_back(coefficients);
            all_surfaces.push_back(inliers_cloud);
      
            ///prepare for next iter
            cloud = outliers_cloud;
            i++;
        }

  

    
    
    //getting the inliers/outliers cloud given an pointcloud and indices
    pcl::PointCloud<PointT>::Ptr filter_cloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr indices, bool negative)
    {
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.setNegative(negative);
        extract.filter(*cloud_filtered);
        return cloud_filtered;
    }

  

    
    
    void do_ransac(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
    {
        pcl::SACSegmentation<PointT> seg;
        //pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        //seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold);
        seg.setMaxIterations(max_iterations);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
    }
    

  

  

