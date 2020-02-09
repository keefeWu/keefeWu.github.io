---
title: 从RCNN到FAST-RCNN、FASTER-RCNN再到MASK-RCNN
date: 2020-02-08 15:11:27
tags:
---
### RCNN

首先RCNN的R是region的意思，也就是region selective search + CNN  
具体解释就是先使用区域搜索方法找出若干个可能是物体的区域,一般通过纹理颜色直方图之类的来挑选出来。然后交给改进版的alexnet来提取特征（这里输入应该是通过resize方法适应网络的），在第五个池化层把输出交给SVM，SVM判断是不是物体，以及物体的种类。  
然后使用一个线性回归对bounding box处理，找出更加紧凑的边界。

### FAST RCNN

之前每次搜索出来的区域都是重新计算特征的，就是resize然后交给网络去计算，现在使用一个cnn，来直接处理原图，使用（roi
pooling）的方法适应输入大小。这样一张图统一提取了特征，然后region selective
search找出区域之后就不需要重新算特征，只需要在相应位置提取就可以了。  
另外FAST RCNN把svm 和
线性回归也放进CNN里了，用了一个新的网络，在最后一层使用softmax代替之前的SVM进行分类以及算出置信度、在softmax平行的地方加上一层线性回归，用来计算边界。

### FASTER RCNN

现在把选择搜索这一步也换成CNN来处理了，在提取特征的网络最后加了一层区域选择网络RPN来代替之前的选择搜索，这样就有四个损失函数了：  
RPN分类 置信度  
RPN回归 范围  
FAST RCNN分类 置信度  
FAST RCNN回归 范围  
所以实际上使用RPN找出大致的范围，使用FAST RCNN算出最终分类以及范围

### MASK RCNN

在FASTER RCNN之前加了一层全卷积网络，将候选区域变成像素级的了。然后再用ROIALIGN对齐。

### SPP

空间金字塔池化（spatial pyramid polling）  
其实就是用一个池化层代替全连接，比方说把图片切成4份，切成8份，这样就有12维特征了。  
而且这样不用关注图片大小，反正就是按比例来分就可以了。 FAST RCNN的roipolling就是借鉴了这个方法做的。

