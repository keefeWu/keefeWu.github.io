---
title: c-CUDA编程《〇》-七个基本步骤
date: 2020-08-17 09:05:20
tags: cuda
---

# 基本步骤
一个完整的cuda程序,需要经历7个基本步骤,如果是单GPU,则可以省略指定显卡的步骤,缩减到5个基本步骤.
* cudaSetDevice
* cudaMalloc
* cudaMemcpy
* kernelFunc
* cudaMemcpy
* cudaFree
* cudaDeviceReset
这里`cudaMemcpy`出现了两遍,第一遍是从cpu拷贝到gpu,第二步是从gpu拷贝到cpu,具体的区别下面有详细讲解

## cudaSetDevice
```
cudaSetDevice(0); 
```
指定gpu的id,如果不设置默认使用0号,所以如果只有一个gpu或者就想使用0号的时候不用执行这个函数.

## cudaMalloc
```
float *aGpu;
cudaMalloc((void**)&aGpu, 16 * sizeof(float));
```
分配显存的函数,先定义一个指针,然后分配显存空间.传入两个参数,第一个是指针地址,第二个是显存大小.

## cudaMemcpy
```
cudaMemcpy(aGpu, a, 16 * sizeof(float), cudaMemcpyHostToDevice);
```
数据拷贝函数,可以把数据从内存拷贝到显存,那个`a`就是cpu上的数据,`aGpu`是分配好了显存上的地址,利用这个函数,可以把数据在内存和显存上拷贝.第三个参数为内存大小,最后一个参数为指定从内存拷贝到显存,如果反方向则是`cudaMemcpyDeviceToHost`
```
cudaMemcpy(b, bGpu, 1 * sizeof(float), cudaMemcpyDeviceToHost);
```

## kernelFunc
```
kernelFunc<<<1, 16>> >(aGpu, bGpu);
```
这个函数是自己定义的,名字也是自己取的,比方说加法就可以叫add,sum都可以.
`<<<1, 16>> >`代表指定1个block,16个线程,圆括号内为函数的参数,参数如果是数组的话一定要是gpu上的地址.


## cudaFree
```
cudaFree(aGpu)
```
释放显存空间

## cudaDeviceReset
```
cudaDeviceReset()
```
重置设备,这步可以省略

# 设备管理函数

## cudaGetDeviceCount
```
int gpuCount = -1;
cudaGetDeviceCount(&gpuCount);
printf("gpuCount: %d\n",gpuCount);
```
查询可用gpu的数量,传入一个int变量的地址,最终结果会写到这个地址上.

## cudaSetDevice
```
cudaSetDevice(0); 
```
指定gpu的id,如果不设置默认使用0号,所以如果只有一个gpu或者就想使用0号的时候不用执行这个函数.

## cudaGetDevice
```
int deviceId;
cudaGetDevice(&deviceId);
printf("deviceId: %d\n",deviceId);
```
获取当前线程只用的gpu的id,结果也是卸载传入的那个地址上.

## cudaGetDeviceProperties
这个函数可以获取gpu的很多信息
```
struct cudaDeviceProp {
char name[256];         //器件的名字
size_t totalGlobalMem;    //Global Memory 的byte大小
size_t sharedMemPerBlock;   //线程块可以使用的共用记忆体的最大值。byte为单位，多处理器上的所有线程块可以同时共用这些记忆体
int regsPerBlock;                 //线程块可以使用的32位寄存器的最大值，多处理器上的所有线程快可以同时实用这些寄存器
int warpSize;                    //按线程计算的wrap块大小
size_t memPitch;        //做内存复制是可以容许的最大间距，允许通过cudaMallocPitch（）为包含记忆体区域的记忆提复制函数的最大间距，以byte为单位。
int maxThreadsPerBlock;   //每个块中最大线程数
int maxThreadsDim[3];       //块各维度的最大值
int maxGridSize[3];             //Grid各维度的最大值
size_t totalConstMem;  //常量内存的大小
int major;            //计算能力的主代号
int minor;            //计算能力的次要代号
int clockRate;     //时钟频率
size_t textureAlignment; //纹理的对齐要求
int deviceOverlap;    //器件是否能同时执行cudaMemcpy()和器件的核心代码
int multiProcessorCount; //设备上多处理器的数量
int kernelExecTimeoutEnabled; //是否可以给核心代码的执行时间设置限制
int integrated;                  //这个GPU是否是集成的
int canMapHostMemory; //这个GPU是否可以讲主CPU上的存储映射到GPU器件的地址空间
int computeMode;           //计算模式
int maxTexture1D;          //一维Textures的最大维度  
int maxTexture2D[2];      //二维Textures的最大维度
int maxTexture3D[3];      //三维Textures的最大维度
int maxTexture2DArray[3];     //二维Textures阵列的最大维度
int concurrentKernels;           //GPU是否支持同时执行多个核心程序
}
```
传入参数是`cudaDeviceProp`类型的结构体对象的地址,最后会写给这个对象,我们随便打印几个
```
cudaDeviceProp prop;
cudaGetDeviceProperties(&prop, 0);
printf("maxThreadsPerBlock: %d\n",prop.maxThreadsPerBlock);
printf("maxThreadsDim: %d\n",prop.maxThreadsDim[0]);
printf("maxGridSize: %d\n",prop.maxGridSize[0]);
printf("totalConstMem: %d\n",prop.totalConstMem);
printf("clockRate: %d\n",prop.clockRate);
printf("integrated: %d\n",prop.integrated);
```

## cudaChooseDevice
```
cudaChooseDevice(&deviceId, &prop);
printf("deviceId: %d\n",deviceId);
```
这个函数十分方便,通过刚才读取出来的prop对象,自动选择一个最合适的设备id,最后结果放在第一个参数的地址里.

## cudaSetValidDevices
```
int deviceList[2] = {0, 1};
cudaSetValidDevices(deviceList, 1)
```
这个函数可以同时设置一串可用的gpu设备,把设备id作为数组传递进去,第一个参数就是设备列表,第二个参数是设备数量,返回值是cuda的ERROR判断机制,写了一个程序捕捉这个错误.

```
static void HandleError(cudaError_t err,
                        const char *file,
                        int line)
                        {
                            if(err != cudaSuccess)
                            {
                                printf("%s in %s at line %d\n",
                                cudaGetErrorString(err),
                                file, line);
                                exit(EXIT_FAILURE);
                            }
                        }
#define HANDLE_ERROR(err) (HandleError(err, __FILE__, __LINE__))
```
程序会从你给的这个列表里的数据按顺序分配,如果你指定的设备数量是0的话就分配默认的,如果你给的这个列表里有无效的id,那么会返回异常.

## cudaMallocPitch
```
size_t width=10;
size_t height=10;

float* decPtr;
size_t pitch;
cudaMallocPitch((void**)&decPtr,&pitch,width*sizeof(float),height);  
```
自动2D对齐的cpu内存分配,如果是二维矩阵的计算,强烈建议使用这个函数来分配.
具体点解释呢,这就涉及到内存对齐的问题了,简单点说,如果是二维数组,那么要保证每行的开始那个数据的地址要是一个batch的整数倍,例如我的电脑是512(128*4,这里的4是float类型的大小,也就是一个batch存128个数),也就是这样才能最高效率的读取.这个batch是可以直接打印`pitch`就能看到的.
在执行`cudaMallocPitch`函数的时候,一个`pitch`就是一行的大小.例如,我们设置
```
size_t cols = 522;
size_t rows = 16;
std::cout<<"pitch:"<<pitch<<std::endl;
std::cout<<"occupy_num:"<<pitch/sizeof(float)<<std::endl;
```
那么打印的结果是
```
pitch:2560
occupy_num:640
```
这里我们分配了`2560`的大小,也就是说`523` `524` ... `640`这些列数都会分配这么多大小,这样才能保证下一行行首对齐
假设`cols`改成1,那么结果是
```
pitch:512
occupy_num:128
```
结果就是一个基本单元`512`,也就是说这里是以128个数为一次分界,每次都会这样对齐.
取下一行的时候用这一行的首地址+pitch就可以了
`cudaMallocPitch`分配的空间可以用`cudaFree`释放

## cudaMalloc3D
`cudaMallocPitch`的3D版,传的参数有所区别,就是把行列那些写入了一个结构体,再把结构体传入
```
cudaExtent extent = make_cudaExtent(sizeof(float) * x, y, z);
cudaPitchedPtr d_data;
cudaMalloc3D(&d_data, extent);

dim3 threads_per_block = dim3(32, 32, 1);
dim3 blocks_per_grid = dim3(32, 1, 1);
extract_patches_from_image_data<<<blocks_per_grid, threads_per_block>>>(d_data);
```
他对应的内存拷贝也有专门的函数`cudaMemcpy3D`
```
cudaMemcpy3DParms cpyParm;
cpyParm = {0};
cpyParm.srcPtr = make_cudaPitchedPtr((void*)h_data, sizeof(float) * width, width, height);
cpyParm.dstPtr = d_data;
cpyParm.extent = extent;
cpyParm.kind = cudaMemcpyHostToDevice;
cudaMemcpy3D(&cpyParm);
```
## cudaMemcpyPeer
GPU变量之间传递数据的函数
```
__host__ ​cudaError_t cudaMemcpyPeer ( void* dst, int  dstDevice, const void* src, int  srcDevice, size_t count )
```
他可以直接被`cudaMemcpy`替代,但是他的优势在于可以指定设备id,假设你只有一块gpu,那么设备id都是0,用哪个都一样,如果是不同设备的话,就需要用这个函数,来指定不同的设备.他的第二个参数和第四个参数分别指定了目标和源头的设备id,第五个参数是数据长度.就是malloc的那个长度.
记得使用前还是要在device上分配内存大小的,这个函数只管拷贝,不管分配的.

## cudaThreadSynchronize
线程同步函数
```
__host__ ​cudaError_t cudaThreadSynchronize ( void ) 
```
这个函数是在gpu的核函数里用的,没有参数,他的作用就是等待所有的线程都执行到这一步,这个函数在很多时候需要一边处理一边读其他线程处理结果的时候会用到,例如归约求和,就是不断的求和,写入新的地址,然后下一轮就要用到上一轮算出的这个结果.

