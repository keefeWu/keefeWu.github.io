---
title: caffe测试程序《一》softmax
date: 2020-02-08 15:11:03
tags:
---
最近训练了一个人脸识别模型，使用的是VGGFace2，使用了1000类来训练，今天介绍一下训练好的模型如何使用。

首先是deploy文件，注意这个deploy不是train.prototxt，一定是单独的deploy.prototxt，他的开头和结尾和train.prototxt是不一样的。

具体在于data层，deploy的data层就是input

    
    
    input: "data"
    input_dim: 1
    input_dim: 3
    input_dim: 112
    input_dim: 96

这样就指定好了，不需要大括号，规定一下这是输入层，以及他的维度就可以了。

然后结尾把train.prototxt里的accuracy，还有softmaxwithloss层删掉，加上一个softmax层

    
    
    layer {
      name: "prob"
      type: "Softmax"
      bottom: "my_fc6"
      top: "prob"
    }

把这个加在最后，deploy文件就算完成了。

然后开始重点，介绍python代码部分，c++的后续我也会补上

首先不需要太多的import，一个caffe就够了

    
    
    import caffe 
    

接着定义模型文件和权重文件的路径

    
    
    deploy = 'deploy.prototxt'
    caffe_model = 'models_1000/_iter_16000.caffemodel'

接着就可以生成一个网络的对象了

    
    
    net = caffe.Net(deploy,caffe_model,caffe.TEST) 

接着生成一个transformer对象，用来对输入图片进行各种转换，包括尺度表换，颜色通道变换等等

    
    
    transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape}) 
    
    
    transformer.set_transpose('data', (2,0,1)) 
    transformer.set_channel_swap('data', (2,1,0))

第一行是转换数据格式，把原本的h, w, c换成c, h, w,也就是把颜色通道提前，这是caffe要求的格式

第二行是把颜色通道内部顺序交换，原本读进来的是bgr通道，改成caffe需要的rgb通道

    
    
    img_path = 'test.jpg'
    im=(caffe.io.load_image(img_path)) * 2 - 1

然后读取图片，使用的是caffe自带的io读取，读进来是bgr通道的，并且是浮点型0~1大小范围的，我训练的时候是归一化到-1~1了，所以需要*2-1，这里酌情缩放和减均值

    
    
    net.blobs['data'].data[...] = transformer.preprocess('data',im)

这一步是把刚才读好的图片喂给caffe的输入层

    
    
    out = net.forward() 

然后就开始执行了，结果赋值给out变量

    
    
    prob = net.blobs['prob'].data[0].flatten()

取出最后softmax概率那一层，把它拉平赋值给prob变量

    
    
    print prob.argmax()

打印最大的那个概率的id，也就是猜测类别的那个对应id了

******************************************************************填坑时间**********************************************************************

这里之前遇到了一个坑，我不知道deploy里面要自己添加一个softmax层，看到最后一个全连接层输出个数就是分类个数，以为那一层就是softmax，最后打印出来的那一层的的输出还有负数，显然不是概率，实际的做法是最后还需要加入一个softmax层，问题就都解决了。

完整代码：

    
    
    import caffe 
    deploy = 'deploy.prototxt'
    caffe_model = 'models_1000/_iter_16000.caffemodel'
    net = caffe.Net(deploy,caffe_model,caffe.TEST)
    transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape}) 
    transformer.set_transpose('data', (2,0,1)) 
    transformer.set_channel_swap('data', (2,1,0))
    img_path = '/data/dataset/VGGFace2/train_align/n000018/0050_02.png'
    im=(caffe.io.load_image(img_path)) * 2 - 1
    net.blobs['data'].data[...] = transformer.preprocess('data',im)
    out = net.forward() 
    prob = net.blobs['prob'].data[0].flatten()
    print prob.argmax()

