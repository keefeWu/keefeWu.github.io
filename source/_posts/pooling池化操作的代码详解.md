---
title: pooling池化操作的代码详解
date: 2020-02-08 15:21:40
tags:
---
池化这个操作原理非常简单，相信大家都很容易搞懂，那么这篇博客就主要从代码来谈谈吧。

首先还是简单介绍一下原理（不是我偷懒想要简单介绍，是因为这个东西介绍起来本身就很简单![偷笑](0.gif)）

所谓的池化目的就是抽象，把一大片面积总结成一个数，比方说一副100*100的图，我们每50*50的区域就取一个最大值，共计能取4个最大值，那么这个2*2的最大值图像就是池化后的图像了。

![](1.png)

上图是一个20*20的图经过size为10步长也为10的池化，最后生成了一个2*2的图。其实最后生成的图与池化的size并没有什么直接关系，真正影响他的是池化的步长，也就是说这一操作不仅可以像上图那样紧挨着移动，还可以隔着几格移动，例如步长如果是11那么就是隔着一格移动了。当然也可以重叠的移动，例如步长是5那么每两个中间就重叠了五格。至于最后如果有出界的情况怎么办。我采取的办法是把出界的部分延长，当0来考虑，反正是求最大值，相当于就没有考虑他们。

这样的话池化结果的大小应该就是原图的大小/步长，如果是小数并且按我的延长方法处理的话就进一，如果选择舍弃的话就取整。

附上代码：

    
    
    Mat pooling(Mat img, int grid, int overlap)
    {
    	Mat pool_img = Mat((int)((img.rows - 1) / overlap) + 1, (int)((img.cols - 1) / overlap)+1, CV_8UC1);
    	for (int col = 0,pool_col=0; col < img.cols; col+= overlap)
    	{
    		for (int row = 0,pool_row=0; row < img.rows; row+= overlap)
    		{
    			int minCol = min(col + overlap, img.cols);
    			int maxData = 0;
    			for (int poolX = col; poolX < minCol; poolX++)
    			{
    				int minRow = min(row + overlap, img.rows);
    				for (int poolY = row; poolY<minRow; poolY++)
    				{
    					if (img.at<uchar>(poolY, poolX)>maxData)
    					{
    						maxData = img.at<uchar>(poolY, poolX);
    					}
    				}
    			}
    			pool_img.at<uchar>(pool_row, pool_col) = maxData;
    			pool_row++;
    		}
    		pool_col++;
    	}
    	return pool_img;
    }

代码是使用opencv写的，实现起来也非常简单。一共传入3个参数，分别是原图，池化的size，最后一个是步长。

可以看到

    
    
    Mat pool_img = Mat((int)((img.rows - 1) / overlap) + 1, (int)((img.cols - 1) / overlap)+1, CV_8UC1);

  
这一句就是按之前说的方法定义池化后的图像，大小就是用的原图大小/步长然后进一。为什么我要先-1呢，这是因为如果不-1那么后面进一操作在刚好能整除的时候就多了1了。就像图中的例子，20/10=2，刚好能整除，如果直接+1就会多出来一格，所以先-1再除再加上1能很好的应对这个问题。

接着就是按照步长遍历原图，每一个区域都寻找最大值，然后赋值给新的结果。

