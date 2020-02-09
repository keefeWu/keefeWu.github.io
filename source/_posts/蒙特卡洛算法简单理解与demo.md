---
title: 蒙特卡洛算法简单理解与demo
date: 2020-02-08 15:19:58
tags:
---
所谓蒙特卡洛算法，实际上就是用频率估计概率。

首先我们知道一个边长为2的正方形面积是2*2=4，他的内接圆的面积是π*1，那么我们在这样一个正方形内随机生成10000个点，落在圆里的点的个数/10000就应该是π/4，所以我们可以估计出π是落在圆里的点的个数*4/10000.

直接看程序:

    
    
    import numpy as np
    
    radius=1
    test_num=100000
    #random generate 10000 points in a sqare width is 2,and the center is in origin
    sqare_x=2*np.random.random_sample(test_num)-1
    sqare_y=2*np.random.random_sample(test_num)-1
    incircle_point_num=0
    #count how many points in the incircle
    for point_count in range(len(sqare_x)):
        if(sqare_x[point_count]*sqare_x[point_count]+
                   sqare_y[point_count]*sqare_y[point_count]<
                   radius*radius):#judge if the distance lower than radius
            incircle_point_num+=1
    print 'PI: '+str(4.0*incircle_point_num/test_num)

  
这里随机生成了10000个在边长为2，中心在原点的正方形中的点，根据坐标判断是否在内接圆内，统计内接圆内点的个数，最后根据公式求出π。

准确度基本上随着样本个数增加而增加，下面是测试的一组统计：

测试个数 |  算出的π的值  
---|---  
10 | 4.0  
100 | 3.2  
1000 | 3.192  
10000 | 3.1388  
100000 | 3.13728  
1000000 | 3.142496  
10000000 | 3.14147268  
|  
|  
  
再测下去计算太慢，也没有意义了，毕竟我不是真的想知道π是多少。

根据统计结果可以看出来，测试样本越大，精确度也就越高，频率也就越接近概率值。

所以这套蒙特卡洛方法可以用来求出积分，也是使用积分区域的面积和外接长方形的面积比的方式估算。

例如：

求

∫10e−x2/2/√2πdx∫01e−x2/2/2πdx

可以知道这是一个单调递减函数，最大值就是x=0的时候也就是1/√2π

π，最小值是x=1的时候，也就是1/e^(0.5)/√2π

那么外接长方形面积就应该是1/√2π-1/e^(0.5)/√2π

所以积分面积=长方形面积*积分内点的个数/样本个数

    
    
    import numpy as np
    import math
    #calculate the function
    def f(x):
        return math.exp(-1*x*x/2)/math.sqrt(2*math.pi)
    
    test_num=10000
    maxy=1/math.exp(0.5)/math.sqrt(2*math.pi)#calculate the top bound
    rectangle_y=np.random.random_sample(test_num)*(1/math.sqrt(2)-maxy)
    rectangle_x=np.random.random_sample(test_num)
    inarea_point_num=0
    #count how many points in the area
    for point_count in range(len(rectangle_x)):
        if(rectangle_y[point_count]<f(rectangle_x[point_count])):
            inarea_point_num+=1
    print 'area: '+str(1.0*inarea_point_num/test_num*(1/math.sqrt(2)-maxy) )

  
求出来的结果是0.340433079875，与准确值相差无几  

