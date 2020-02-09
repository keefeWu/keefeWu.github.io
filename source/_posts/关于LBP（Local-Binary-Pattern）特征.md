---
title: 关于LBP（Local-Binary-Pattern）特征
date: 2020-02-08 15:22:05
tags:
---
LBP是英文Local Binary
Pattern的缩写，顾名思义，他是一个局部的二值特征。所谓局部，就是他的提取方法是根据他附近点的信息来计算的，所谓二值，就是特征只有0和1了。

具体操作其实也非常简单，就是做一个判断的滤波器，什么叫判断的滤波器呢？就是判断附近的点是不是比中间的点大，如果大就是1，小就是0，等于的话就看规定了，可以规定0也可以规定1，但是规定一定要统一，不能一会是0一会是1，（不过这样变得代码也不好写啊，估计没有哪个会神经的让它不统一吧）。

![](0.png)![](1.png)  

如上两张图，图一是原图矩阵，图二是LBP处理后的矩阵。就拿画红框的区域来分析，中间的点是194，那么从左上必到右下依次是10100100（其中大于等于用1表示，小于用0表示），转换成十进制就是164，所以处理后的那个位置就是164了。

由于想要保持处理后的图和原图一样大小，对于出界区域我用的是边界的值，处理前后的对比图如下图：

![](2.png)![](3.png)

左边为原图，右边为特征图。可以看到轮廓特征非常鲜明。

代码如下：

    
    
    void uniformLBP(Mat &image, Mat &result)
    {
    	int height = image.rows;
    	int width = image.cols;
    	for (int y = 0; y < height; y++)
    	{
    		for (int x = 0; x < width; x++)
    		{
    			uchar neighbor[8] = { 0 };
    			neighbor[0] = image.at<uchar>(mid(y + 1, 0, height - 1), mid(x + 1, 0, width - 1));
    			neighbor[1] = image.at<uchar>(mid(y + 1, 0, height - 1), mid(x, 0, width - 1));
    			neighbor[2] = image.at<uchar>(mid(y + 1, 0, height - 1), mid(x - 1, 0, width - 1));
    			neighbor[3] = image.at<uchar>(mid(y, 0, height - 1), mid(x + 1, 0, width - 1));
    			neighbor[4] = image.at<uchar>(mid(y, 0, height - 1), mid(x - 1, 0, width - 1));
    			neighbor[5] = image.at<uchar>(mid(y - 1, 0, height - 1), mid(x + 1, 0, width - 1));
    			neighbor[6] = image.at<uchar>(mid(y - 1, 0, height - 1), mid(x, 0, width - 1));
    			neighbor[7] = image.at<uchar>(mid(y - 1, 0, height - 1), mid(x - 1, 0, width - 1));
    			uchar center = image.at<uchar>(mid(y, 0, height - 1), mid(x, 0, width - 1));
    			uchar temp = 0;
    			for (int k = 0; k < 8; k++)
    			{
    				temp += (neighbor[k] >= center)* (1 << k);  // 计算LBP的值  
    			}
    			//cout << setw(3) << (int)temp << " ";
    			result.at<uchar>(y, x) = temp;   
    			
    		}
    		//cout << endl;
    	}
    }

  

LBP等价模式（降维到59维）

我们看到的LBP特征都是0-255共256维，因为八个数2的八次方种情况就是256了。其实如果把他们归类一下还可以降维到59种，当然了，想降到几种都可以，比方说降成两种，就可以规定奇数是一种，偶数是一种。我们今天讲的这种59种的方法叫做LBP等价模式。他的分法是根据那8位二进制数，他们的过程来决定的。比方说01000010，其中从0到1这个过程一共出现了2次，从
1变成0这个情况也发生了两次，那么就可以说他是一个4次跳变（2+2=4）。由于绝大多数的数字的跳变次数都在2以内，所以我们就把跳变次数小于等于2的叫做等价模式，大于2的叫做非等价模式。其中等价模式一共有58种情况，分别是。。。我就不分别列了。。。。再加上一个非等价模式一共59种。

  

![](1.png)![](5.png)

如上图，可以看到164是10100100一共跳变了6次（首尾循环），所以属于非等价模式，而旁边的255跳变了0次，240是01110000跳变了两次，属于等价模式，所以他们范围都在0-58之间。这样简化以后的效果如图：

![](3.png)![](7.png)

左边就是上面的普通LBP特征，右边是降维后的，可以看到除了变暗以外没太大区别，变暗是因为数字变小了，只是视觉上的效果，从另一面也说明了亮度范围变小了，确实起到降维的作用了。下面附上判断跳变次数的代码：

    
    
    // 计算跳变次数  
    int getHopCount(int i)
    {
    	int a[8] = { 0 };
    	int cnt = 0;
    	int k = 7;
    	while (i)
    	{
    		a[k] = i & 1;
    		i = i >> 1;
    		--k;
    	}
    	for (k = 0; k < 7; k++)
    	{
    		if (a[k] != a[k + 1])
    		{
    			++cnt;
    		}
    	}
    	if (a[0] != a[7])
    	{
    		++cnt;
    	}
    	return cnt;
    }
    

  
另外，我完整的项目代码会发布在github上，欢迎fork：

https://github.com/DigitalImageProcessingBlog/LBP.git  

