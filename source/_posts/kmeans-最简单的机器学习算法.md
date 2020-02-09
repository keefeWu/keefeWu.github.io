---
title: kmeans(最简单的机器学习算法)
date: 2020-02-08 15:21:48
tags:
---
<http://blog.csdn.net/u011021773/article/details/53959108>  

Kmeans算法是机器学习最经典最简单的一种算法，它属于无监督学习类，又叫做聚类。是指没有给出正确答案，只是自己根据相关特性进行分类的一种方法，学习的目的就是找出最相关的特性。

Kmeans可以很形象的用图形变现出来，是个理解无监督学习非常直观的例子。

随机给出二十个点，我们要做的就是把这二十个点分成n类。如图所示：

![](0.png)

![](1.png)  

  

上面图的白圈就是我们随机生成的点，可以看到先面图根据他们的位置自动分成了三类，标注成了三种颜色。

我们再来上一张分成四类的图：

![](2.png)  

![](3.png)  

好了，效果就摆在这了，具体是怎么实现的呢？

可千万不要看看热闹就走了，不敢往下看了，因为原理非常的简单，可以很快的让人理解。

首先确定我们的目标是把这20个随机的点分成n类，那么无非就是找到n个中心，然后每个点和这n个中心比较，离谁近就是哪一类了。举个例子，我们先随机选择n个点的坐标作为中心（假设取前n个点分别作为n类的中心），对于第一个点，他与每一个中心计算距离（这里的距离是真实的空间距离，也就是横纵坐标的差的平方和(x1-x）^2+(y1-y)^2),每个点都算出n个距离，然后选择距离最近的那一类作为自己的类。每个点都分好类以后再对每一类的坐标进行平均，作为新的中心，再重复之前的分类过程，分好类后再找出新的中心。这样迭代若干次（我这里用的100次），就可以出来一个比较可观的分类了。

不过kmeans找到的并不是最优解，每一迭代结果也都在变，起点选的不一样结果也不一样，但是找到的都是分类比较好的结果，在无监督学习时候的分类有着很大的作用。

最后附上代码：

    
    
    #include "iostream"
    #include "opencv2\imgproc.hpp"
    #include "highgui.h"
    #include "opencv2\opencv.hpp"
    #include "time.h"
    using namespace std;
    using namespace cv;
    struct Color {
    	uchar r, g, b;
    };
    
    //background:背景图
    //num需要分类的个数
    bool kmeans(Mat &bak, int num, int x_array[], int y_array[])
    {
    	
    	//选择前num个点作为分类起点
    	int *start_x = new int[num];
    	int *start_y = new int[num];
    	int *sum_x = new int[num];//记录每一类的坐标总和
    	int *sum_y = new int[num];
    	int *class_num = new int[num];//记录每一类的个数
    	int class_array[20];//记录每个点的类别
    	for (int i = 0; i < num; i++)
    	{
    		start_x[i] = x_array[i];
    		start_y[i] = y_array[i];
    		class_array[i] = i;
    	}
    	//迭代100次
    	for (int count = 0; count < 100; count++)
    	{
    		for (int i = 0; i < num; i++)
    		{
    			sum_x[i] = 0;
    			sum_y[i] = 0;
    			class_num[i] = 0;
    		}
    
    		for (int i = 0; i < 20; i++)
    		{
    			int dist_min = -1;//寻找最近距离类别
    			for (int j = 0; j < num; j++)
    			{
    				if (dist_min < 0 || dist_min >(x_array[i] - start_x[j])*(x_array[i] - start_x[j]))
    				{
    					class_array[i] = j;
    					dist_min = (x_array[i] - start_x[j])*(x_array[i] - start_x[j]);
    				}
    			}
    			sum_x[class_array[i]] += x_array[i];
    			sum_y[class_array[i]] += y_array[i];
    			class_num[class_array[i]]++;
    		}
    		//计算聚类出来的新中心
    		for (int i = 0; i < num; i++)
    		{
    			start_x[i] = sum_x[i] / class_num[i];
    			start_y[i] = sum_y[i] / class_num[i];
    			sum_x[i] = 0;
    			sum_y[i] = 0;
    		}
    	}
    	//设置每种类别的颜色
    	Color *MyColor = new Color[num];
    	srand((int)time(NULL));
    	for (int i = 0; i < num; i++)
    	{
    		
    		MyColor[i].r = (uchar) rand() % 256;
    		MyColor[i].g = (uchar) rand() % 256;
    		MyColor[i].b = (uchar) rand() % 256;
    		
    	}
    	Mat background = Mat::zeros(Size(640, 320), CV_8UC3);
    	imshow("bak", background);
    	waitKey(0);
    	//绘制分类后的图形
    	for (int i = 0; i < 20; i++)
    	{
    		cout << class_array[i] << ": "<<x_array[i] << "," << y_array[i] << "          "<< (int)MyColor[class_array[i]].r <<" "<<(int) MyColor[class_array[i]].g << " " << (int)MyColor[class_array[i]].b << " " << endl;
    		//circle(background, Point(x_array[i], y_array[i]), 5, CV_RGB(255, 0,255),-1);
    		circle(background, Point(x_array[i], y_array[i]), 5, CV_RGB((int)MyColor[class_array[i]].r, (int)MyColor[class_array[i]].g, (int)MyColor[class_array[i]].b), -1);
    	}
    	imshow("kmeans", background);
    	waitKey(0);
    	bak = background;
    	return true;
    }
    bool drawCircle(Mat &background, int x_array[],int y_array[])
    {
    	
    	srand((int)time(NULL));
    	//随机产生坐标
    	for (int i = 0; i < 20; i++)
    	{
    		x_array[i] = rand() % background.cols;
    		y_array[i] = rand() % background.rows;
    		//防止有画重的圆圈
    		for (int j = 0; j < i; j++)
    		{
    			while (x_array[i] == x_array[j] && y_array[i] == y_array[j])
    			{
    				x_array[i] = rand() % background.cols;
    				y_array[i] = rand() % background.rows;
    			}
    		}
    		//绘制圆形
    		circle(background, Point(x_array[i], y_array[i]), 5, CV_RGB(255, 255, 255),-1);
    	}
    	return true;
    
    }
    void main()
    {
    	Mat background = Mat::zeros(Size(640, 320), CV_8UC3);
    	int x_array[20], y_array[20];
    	drawCircle(background, x_array, y_array);
    	imshow("background",background);
    	waitKey(0);
    	imwrite("../image/background2.jpg", background);
    	kmeans(background, 4, x_array, y_array);
    	imshow("kmeans", background);
    	waitKey(0);
    	imwrite("../image/kmeans2.jpg", background);
    }
    

  

