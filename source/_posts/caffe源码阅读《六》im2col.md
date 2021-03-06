---
title: caffe源码阅读《六》im2col
date: 2020-03-04 21:16:25
top: 6
tags: [caffe]
categories: 
- caffe源码解读
---
# 卷积的过程
卷积实际上就是两个矩阵对位相乘，然后再把积求和的一个过程
![](https://blog.357573.com/2020/02/08/%E4%BB%80%E4%B9%88%E6%98%AF%E5%8D%B7%E7%A7%AF%E5%91%A2/0.png)
# im2col
caffe里的卷积运算实际上是把待输入图像和卷积核都转换成矩阵，然后通过矩阵的乘法一步得出来的。这里列举一个例子，假设我们待输入图像是 <code>5*5</code>的一张图
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/1.jpg)
为了方便看清位置，我们就用12345来表示图像的值。
我们选择 <code>3*3</code>的卷积核
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/2.jpg)
然后和图像对位相乘，也就是卷积核的 <code>1</code>的位置的值和图像的 <code>1</code>的位置的值相乘，卷积核的 <code>2</code>的位置的值和图像的 <code>2</code>的位置的值相乘。。。
这样就完成了卷积的第一步。
## 原图转化成矩阵
那么如果我们把第一次的这9个值写成一列的话就是：
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/3.jpg)
那么这9个值就对应的是我们原图上的这9个格子
卷积核每往后移动一步，就把新的9个值加在右边。
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/4.jpg)
最后一列一列的加，把整张图所有的计算要用到的数据都写过去，到时候可以直接用个矩阵的乘法一步到位了，做好以后效果如图
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/5.jpg)
## 卷积核转化成矩阵
接下来我们把卷积核也这样转化成矩阵，就可以直接用一个矩阵乘法一步到位了。
因为卷积核对图像来说是左乘，所以卷积核要按行写。
因为核的值是固定的，所以实际上它只有一行，就是这样。
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/6.jpg)
然后它将来要和刚刚生成的那个原图转化后的矩阵相乘的，可以验证一下它的行数等于那个矩阵的列数，这是必然的，因为我们那个原图的转化矩阵就是根据卷积核的大小写的，所以尺寸肯定是匹配的。
## 矩阵乘法求卷积结果
然后这两个矩阵直接做矩阵的乘法就可以了，我们想想矩阵的乘法不就是做这个事的吗，先乘后加。
最后就得出了一行九列的乘法结果
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/7.jpg)
## 将计算矩阵转回特征图
因为卷积之后得到的结果应该是一个特征图，而不会是这样一个行矩阵，所以我们还是要把他转换回去，那么根据计算好的尺寸，变回它应有的大小
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/8.jpg)
## 多通道的输入
如果是RGB3个通道的卷积，在caffe里面是3个通道卷积的结果再叠加起来，既然是加法，那么我们就可以直接在原图转化矩阵把每个通道往下叠加，就像这样。
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/9.jpg)
这样的话，矩阵乘法他就能自动把每个通道全部加起来了。但是我们光乘号右边的矩阵行变多了，那么左边矩阵的列也应该变多，所以我们把卷积核也这样复制一下。
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/10.jpg)
就像这样，然后再乘，得出来的结果还是和刚才一样的形状，因为输入多通道只是把结果相加，并不改变形状，这样保证了输出的通道数不变。
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/11.jpg)
## 多通道的输出
如果我们也想要多通道的输出该怎么办呢？比方说我们要从3通道的输入卷积之后生成4通道的输出，那是不是要循环4次这种操作呢？答案是否定的，我们只需要把这4个卷积核按行写在下面就可以了。
我们这里先假设这4个卷积核是一样的，那么我们直接把卷积核的行复制成4行就可以了，当然，实际生产中这4个核肯定是不一样的，不然干嘛要输出4个通道呢
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/12.jpg)
最后的结果也变成了4行了
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/13.jpg)
再看看全部的过程
![](https://blog.357573.com/2020/03/04/caffe源码阅读《六》im2col/14.jpg)
我们最后再把这个矩阵转回featuremap该有的形状就可以了。这样就是caffe做卷积的全部流程，把图都转成一个矩阵，这样直接做一次矩阵乘法就可以了。再gpu这种设备上，牺牲一点内存，使所有计算单元执行同样的计算这样效率是最高的。



