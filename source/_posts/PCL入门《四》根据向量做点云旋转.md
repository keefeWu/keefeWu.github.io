---
title: PCL入门《四》根据向量做点云旋转
date: 2020-02-08 15:19:17
tags:
---
如果已知旋转前某个向量n1(x1,y1,z1)和旋转后的向量n2(x2,y2,z2)实际上已知了三个点，(0,0,0) (x1,y1,z1)
(x2,y2,z2)
那么可以求出他们构成的平面的法向量，也就是同时垂直于n1和n2的这个向量，并且经过n1和n2的交点，也就是原点。这个向量就是他们的旋转轴。

![](0.png)

那么已知三点怎么求平面的法向量呢，我们可以代入公式

![](1.png)

其中x3,y3,z3我们设为原点，那么就是0,0,0.

    
    
    void MatrixRotation::calculate_axis()
    {
      float x1,x2,y1,y2,z1,z2;
      x1=normal_start_.normal_x,x2=normal_end_.normal_x;
      y1=normal_start_.normal_y,y2=normal_end_.normal_y;
      z1=normal_start_.normal_z,z2=normal_end_.normal_z;
      float x3,y3,z3; 
      x3=y3=z3=0;
      axis_.normal_x=(y2-y1)*(z3-z1)-(y3-y1)*(z2-z1);
      axis_.normal_y=(z2-z1)*(x3-x1)-(z3-z1)*(x2-x1);
      axis_.normal_z=(x2-x1)*(y3-y1)-(x3-x1)*(y2-y1);
      //normalize
      float square_sum=axis_.normal_x*axis_.normal_x+
                        axis_.normal_y*axis_.normal_y+
                        axis_.normal_z*axis_.normal_z;
      square_sum=sqrt(square_sum);
      axis_.normal_x/=square_sum;
      axis_.normal_y/=square_sum;
      axis_.normal_z/=square_sum;
    }

  
  

求出了法向量之后带入罗德里格斯旋转公式，求出旋转矩阵，

![{\\displaystyle R={\\begin{bmatrix}\\cos \\theta +u_{x}^{2}\\left\(1-\\cos
\\theta \\right\)&u_{x}u_{y}\\left\(1-\\cos \\theta \\right\)-u_{z}\\sin
\\theta &u_{x}u_{z}\\left\(1-\\cos \\theta \\right\)+u_{y}\\sin \\theta
\\\\u_{y}u_{x}\\left\(1-\\cos \\theta \\right\)+u_{z}\\sin \\theta &\\cos
\\theta +u_{y}^{2}\\left\(1-\\cos \\theta \\right\)&u_{y}u_{z}\\left\(1-\\cos
\\theta \\right\)-u_{x}\\sin \\theta \\\\u_{z}u_{x}\\left\(1-\\cos \\theta
\\right\)-u_{y}\\sin \\theta &u_{z}u_{y}\\left\(1-\\cos \\theta
\\right\)+u_{x}\\sin \\theta &\\cos \\theta +u_{z}^{2}\\left\(1-\\cos \\theta
\\right\)\\end{bmatrix}}.}](https://wikimedia.org/api/rest_v1/media/math/render/svg/f259f80a746ee20d481f9b7f600031084358a27c)

    
    
    void MatrixRotation::matrix_tansform(pcl::PointCloud<PointT>::Ptr cloud)
    {
      Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
      float x,y,z;
      x=axis_.normal_x;y=axis_.normal_y;z=axis_.normal_z;
      transform (0,0)=cos(theta_)+(1-cos(theta_))*x*x;
      transform (0,1)=(1-cos(theta_))*x*y-sin(theta_)*z;
      transform (0,2)=(1-cos(theta_))*x*z+sin(theta_)*y;
      transform (1,0)=(1-cos(theta_))*x*y+sin(theta_)*z;
      transform (1,1)=cos(theta_)+(1-cos(theta_))*y*y;
      transform (1,2)=(1-cos(theta_))*y*z-sin(theta_)*x;
      transform (2,0)=(1-cos(theta_))*x*z-sin(theta_)*y;
      transform (2,1)=(1-cos(theta_))*y*z+sin(theta_)*x;
      transform (2,2)=cos(theta_)+(1-cos(theta_))*z*z;
      pcl::transformPointCloud (*cloud, *cloud, transform);
    }

  
其中，theta是需要旋转的角度，这个公式很简单，搜索求向量夹角就可以了，我这里不再赘述，直接使用了pcl的方法求的，最后使用pcl的旋转，实际上就是现有矩阵乘以旋转矩阵。这样就能得到旋转后的图像了。

完整代码我会上传到github上，敬请期待。。。  

