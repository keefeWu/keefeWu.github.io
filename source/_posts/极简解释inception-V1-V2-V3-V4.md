---
title: 极简解释inception-V1-V2-V3-V4
date: 2020-02-08 15:17:24
tags:
---
### Inception v1

inception的核心就是把google net的某一些大的卷积层换成1*1, 3*3,
5*5的小卷积，这样能够大大的减小权值参数数量。直接上一张完整的图片  
![这里写图片描述](0.png)  
可以看到，卷积了几步之后就开始出现并行的过程，换成几个1*1， 3*3， 5*5的卷积并列。然后再把分别卷积的结果作为层数维度上的合并。  
比方说这一层本来是一个28*28大小的卷积核，一共输出224层，换成inception以后就是64层1*1， 128层3*3，
32层5*5。这样算到最后依然是224层，但是参数个数明显减少了，从28*28*224 = `9834496`
变成了1*1*64+3*3*128+5*5*32 = `2089`，减小了几个数量级。  
再上一张形象的图：  
![这里写图片描述](1.png)  
这个图除了那三个卷积并联，还加上了一个3*3的池化层并联，操作方法一模一样，这里就不多加赘述了。  
换成代码就是

    
    
    def inception(net):
        with tf.variable_scope('Branch_1'):
                tower_conv_1 = slim.conv2d(net, 64, 1, scope='Conv2d_1x1')
        with tf.variable_scope('Branch_3'):
                tower_conv_3 = slim.conv2d(net, 128, 3, scope='Conv2d_3x3')
        with tf.variable_scope('Branch_5'):
                tower_conv_5 = slim.conv2d(net, 32, 5, scope='Conv2d_5x5')
    
        mixed = tf.concat([tower_conv_1, tower_conv_3, tower_conv_5], 3)
    
        return mixed 

既然变小，为什么不直接使用1*1的，而还需要3*3和5*5的呢，其实这样还是为了适应更多的尺度，保证输入图像即使被缩放也还是可以正常工作，毕竟相当于有个金字塔去检测了嘛。

### Inception V2

inception V2
其实在网络上没有什么改动，只是在输入的时候增加了batch_normal，所以他的论文名字也是叫batch_normal，加了这个以后训练起来收敛更快，学习起来自然更高效，可以减少dropout的使用。  
他的normalization的过程是这样的  
![这里写图片描述](2.png)  
其实也是个很简单的正则化处理，这样保证出现过的最大值为1，最小值为0，所有输出保证在0~1之间。  
论文还提到了训练batch_normal的过程，感兴趣的可以深入了解一下  
![这里写图片描述](3.png)  
当然，有了batch_normal并不是完整的inception
V2，官方定义这个叫做V1的加强版，后来google又出了一篇新论文，把5*5的卷积改成了两个3*3的卷积串联，它说一个5*5的卷积看起来像是一个5*5的全连接，所以干脆用两个3*3的卷积，第一层是卷积，第二层相当于全连接，这样可以增加网络的深度，并且减少了很多参数。  
![这里写图片描述](4.png)  
所以就改成这样了。

### Inception V3

inception
V3把googlenet里一些7*7的卷积变成了1*7和7*1的两层串联，3*3的也一样，变成了1*3和3*1，这样加速了计算，还增加了网络的非线性，减小过拟合的概率。另外，网络的输入从224改成了299.

### Inception V4

inception
v4实际上是把原来的inception加上了resnet的方法，从一个节点能够跳过一些节点直接连入之后的一些节点，并且残差也跟着过去一个。  
另外就是V4把一个先1*1再3*3那步换成了先3*3再1*1.  
论文说引入resnet不是用来提高深度，进而提高准确度的，只是用来提高速度的。

大体上就这么多，然后附上论文：  
[v1] Going Deeper with Convolutions, 6.67% test error,
<http://arxiv.org/abs/1409.4842>  
[v2] Batch Normalization: Accelerating Deep Network Training by Reducing
Internal Covariate Shift, 4.8% test error, <http://arxiv.org/abs/1502.03167>  
[v3] Rethinking the Inception Architecture for Computer Vision, 3.5% test
error, <http://arxiv.org/abs/1512.00567>  
[v4] Inception-v4, Inception-ResNet and the Impact of Residual Connections on
Learning, 3.08% test error, <http://arxiv.org/abs/1602.07261>

