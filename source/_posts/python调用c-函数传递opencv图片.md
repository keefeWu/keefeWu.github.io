---
title: python调用c++函数传递opencv图片
date: 2020-02-03 12:39:00
tags: [python, c++]
categories: 
- python
---

首先c或者c++需要编译成动态链接库，在cmake里面把add_executable改成add_library([project_name]  SHARED [xx.cpp])在写c++代码的时候，由于c++有重载功能，所以编译器在编译的时候是会把函数改名的，这个时候就需要强调需要被python调用的函数是以c的方式编译。例如：<code>extern "C"</code>
```
{
float test()
{
    printf("hello cpp");
    return 1;
}
```
}也就是把要调用的代码加上extern "C" 这句非常重要，没他不行，会说找不到函数声明的，因为在编译的时候被改名了。编译好了之后是个.so文件，在python里面直接引用这个文件就可以了，具体操作为import ctypes
ll = ctypes.cdll.LoadLibrary
lib = ll("./lib/libtest.so")这样就加载了动态链接库，lib这个对象就是动态链接库对象了，想用刚才的test函数，直接用lib.test()就可以了。接下来是传参部分，以opencv为例，传递mat图像，然后返回一个浮点数，在c++文件里这样写
```float test(int height, int width, uchar* frame_data)
{
  cv::Mat image(height, width, CV_8UC3);
  uchar* pxvec =image.ptr<uchar>(0);
  int count = 0;
  for (int row = 0; row < height; row++)
  {
    pxvec = image.ptr<uchar>(row);
    for(int col = 0; col < width; col++)
    {
      for(int c = 0; c < 3; c++)
      {
        pxvec[col*3+c] = frame_data[count];
        count++;
      }
    }
  }
  float value = 0.2;
  return value;
}
```
这里传入的是uchar*类型，只能通过指针来传递，然后一个一个元素的赋值。在python里面怎么写呢
```
import ctypes
import cv2
ll = ctypes.cdll.LoadLibrary
lib = ll("./lib/libtest.so")
lib.test.restype = ctypes.c_float
frame = cv2.imread('test.jpg')
frame_data = np.asarray(frame, dtype=np.uint8)
frame_data = frame_data.ctypes.data_as(ctypes.c_char_p)
value = lib.test(frame.shape[0], frame.shape[1], frame_data)
print value
```
这里补充了一句<code>restype</code>，如果不加这一句的话返回的是一个指针，<code>python</code>并不能解析出来，只能打印出他的地址，所以在之前生命<code>test</code>这个函数返回类型是<code>float</code>，这样value才是python能用的<code>float</code>类型<code>ctypes</code>就是把数据转成c语言支持的类型，<code>c_char_p</code>就是<code>char</code>类型的指针，还有其他类型的，比方说<code>c_int</code>，就是<code>int</code>类型，但是没有<code>int</code>指针，只有<code>void</code>指针，具体有哪些可以查一下<code>ctypes</code>，这个很好查的另外，<code>dtype</code>一定是<code>uint8</code>，不然会默认成<code>uint64</code>，那样在读数据的时候就需要每隔8个读一次了，中间7个都是0，我之前就遇到了这个坑，在代码里面写了个%8来判断的。附上之前的博文为什么我要弄个8出来呢，因为根据我的观察传入的数据是8个隔开的，中间7个都是0，这里肯定是数据类型造成的内存占用问题，我暂时没有好的解决办法。这篇文章里面遇到了好多坑，我都没有解决，写的都是最笨的方法，但是绝对有效。那每8个才出现有效数字是怎么发现的呢，其实就是我自己定义了一个简单矩阵传过来一个一个打印出来观察的。