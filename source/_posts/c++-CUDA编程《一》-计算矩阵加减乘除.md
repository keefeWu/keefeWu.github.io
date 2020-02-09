
    #include "calculator.h"
    #include <cuda_runtime.h>
    #include "iostream"
    
    bool InitCUDA()
    {
    	//used to count the device numbers
    	int count; 
    
    	// get the cuda device count
    	cudaGetDeviceCount(&count);
    	// print("%d\n", count);
    	std::cout << "count: " <<count << std::endl;
    	if (count == 0) 
    	{
    		return false;
    	}
    
    	// find the device >= 1.X
    	int i;
    	for (i = 0; i < count; ++i) 
    	{
    		cudaDeviceProp prop;
    		if (cudaGetDeviceProperties(&prop, i) == cudaSuccess) 
    		{
    			if (prop.major >= 1) 
    			{
    				break;
    			}
    		}
    	}
    	// if can't find the device
    	if (i == count) 
    	{
    		std::cout<<"count: "<<count<<" i:"<<i<<std::endl;
    		return false;
    	}
    
    	// set cuda device
    	cudaSetDevice(i);
    
    	return true;
    }
    
    __global__ void add(const float *dev_a,const float *dev_b,float *dev_c)
    {
        int i=threadIdx.x;
        float sub = dev_a[i]-dev_b[i];
        dev_c[i] = sub * sub;
    }
    
    bool calculate_distance_imp(const float* v1, const float* v2, const int length, float* sub)
    {
    	float *dev_v1, *dev_v2, *result;
    	cudaError_t err = cudaSuccess;
    	err=cudaMalloc((void **)&dev_v1, sizeof(float)*length);
    	err=cudaMalloc((void **)&dev_v2, sizeof(float)*length);
    	err=cudaMalloc((void **)&result, sizeof(float)*length);
    	if(err!=cudaSuccess)
    	{
    	 std::cout<<"the cudaMalloc on GPU is failed"<<std::endl;
    	 return false;
    	}
    	cudaMemcpy(dev_v1,v1,sizeof(float)*length,cudaMemcpyHostToDevice);
    	cudaMemcpy(dev_v2,v2,sizeof(float)*length,cudaMemcpyHostToDevice);
    	add<<<1,length>>>(dev_v1,dev_v2,result);
    	cudaMemcpy(sub,result,sizeof(float)*length,cudaMemcpyDeviceToHost);
    	std::cout<<"("<<sub[0]<<","<<sub[1]<<")"<<std::endl;
    	return true;
    }

首先init_cuda用来初始化，检测有没有GPU，有的话就开启。

然后就可以调用calculate这个函数了，我输入了两个向量，输出的是他们每个对应位置的差的平方，方便后续做第二范数。

那么重点就看calculate_distance_img这个函数了，首先建立3个指针，分别是输入的两个以及输出的一个，用来交给gpu来计算，其实就是在gpu上开辟显存空间。

开辟完空间之后开始把数据拷贝进去，数据在v1和v2上，拷贝到刚刚分配的gpu指针dev_v1和dev_v2上，然后就可以执行我们自定义的计算函数了，名字叫做add，实现在上面，也就是对应每个线程如何处理，我们是先加后平方。

声明完之后就取出结果，使用cudaMemcpy从result这个结果里拷贝下来，到sub这个变量里。然后就可以正常使用sub变量的结果了。

其中cmake文件有两个特殊的地方

一个是寻找cuda的库

find_package(CUDA QUIET REQUIRED)

还有一个是生成可执行程序的时候使用的是

cuda_add_executable(demo main.cpp calculator.cu)

