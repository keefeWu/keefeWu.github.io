---
title: c-CUDA编程《〇》-七个基本步骤
date: 2020-08-17 09:05:20
tags:
---

# 序言
一个完整的cuda程序,需要经历7个基本步骤,如果是单GPU,则可以省略指定显卡的步骤,缩减到5个基本步骤.
* cudaSetDevice
* cudaMalloc
* cudaMemcpy
* kernelFunc
* cudaMemcpy
* cudaFree
* cudaDeviceReset
这里`cudaMemcpy`出现了两遍,第一遍是从cpu拷贝到gpu,第二步是从gpu拷贝到cpu,具体的区别下面有详细讲解

# cudaSetDevice
```
cudaSetDevice(0); 
```
指定gpu的id,如果不设置默认使用0号,所以如果只有一个gpu或者就想使用0号的时候不用执行这个函数.

# cudaMalloc
```
float *aGpu;
cudaMalloc((void**)&aGpu, 16 * sizeof(float));
```
分配显存的函数,先定义一个指针,然后分配显存空间.传入两个参数,第一个是指针地址,第二个是显存大小.

# cudaMemcpy
```
cudaMemcpy(aGpu, a, 16 * sizeof(float), cudaMemcpyHostToDevice);
```
数据拷贝函数,可以把数据从内存拷贝到显存,那个`a`就是cpu上的数据,`aGpu`是分配好了显存上的地址,利用这个函数,可以把数据在内存和显存上拷贝.第三个参数为内存大小,最后一个参数为指定从内存拷贝到显存,如果反方向则是`cudaMemcpyDeviceToHost`
```
cudaMemcpy(b, bGpu, 1 * sizeof(float), cudaMemcpyDeviceToHost);
```

# kernelFunc
```
kernelFunc<<<1, 16>> >(aGpu, bGpu);
```
这个函数是自己定义的,名字也是自己取的,比方说加法就可以叫add,sum都可以.
`<<<1, 16>> >`代表指定1个block,16个线程,圆括号内为函数的参数,参数如果是数组的话一定要是gpu上的地址.


# cudaFree
```
cudaFree(aGpu)
```
释放显存空间

# cudaDeviceReset
```
cudaDeviceReset()
```
重置设备,这步可以省略