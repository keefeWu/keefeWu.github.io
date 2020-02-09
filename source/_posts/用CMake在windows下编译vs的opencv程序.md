---
title: 用CMake在windows下编译vs的opencv程序
date: 2020-02-08 15:22:14
tags:
---
网上有很多CMAKE编译opencv的教程，但是到最后具体程序的时候都是采用VS新建工程的方法，这种方法费时费力还不可移植，需要不断地添加库目录，添加链接项。而如果用CMAKE来管理项目，就可以直接把opencv包含进去，移植到另外的机器也很轻松了。

好，话不多说，咱先从编译opencv开始讲起。首先下载源码版的opencv，打开github，搜索opencv，选择第一个。

![](0.png)

进入以后点击release来选择版本

![](1.png)

我们今天选用3.1的版本，如果需要其他版本步骤也一样，下载source版

![](2.png)

解压缩之后就可以看到所有源文件的目录了。

![](3.png)

接着打开CMAKE的GUI工具，在第一行源路径处填写opencv的源文件目录，也就是上图的目录，第二行填写编译后的目录，我通常是在源文件目录里新建一个build目录，你也可以编译到其他地方。然后点击Configure，选择需要生成工程文件的环境，我这里选择的是VS2015的64位，你可以根据你的vs版本来自行选择。

![](4.png)

这里的BUILD_EXAMPLE是编译例子的意思，opencv给了很多实用教程的例子，文件很多，加上很耗时，如果不需要可以把这个勾去掉。点击Configure之后如果见红了就再点一下，直到是黑色为止，如果一直是红色说明报错了，详情请参照报的错误更正，通常是路径问题。全黑以后再点击Generate，如果全部顺利最后出现的画面是这样的。

![](5.png)  

  

这个就标志着工程文件生成好了，接着打开VS，在build里面打开这个工程Opencv.sln。

![](6.png)

打开以后是这样的，我们右键点击INSTALL把它设为启动项，选择Debug版还是Release版，然后右键开始生成，如果报错请根据错误自行更正，如果进展顺利等十几分钟就成功啦。如果Debug和Release都需要就生成好了以后切换版本再来一次就好了。接下来就可以自己写程序调用opencv了。

在写之前需要把编译好的opencv添加到环境变量里面，因为cmake会自动寻找叫opencv_dir的环境变量，所以我们新建一个opencv_dir，把build目录添加进去就可以了。

![](7.png)  

首先新建一个目录叫src，专门用来存放源码，然后再新建一个目录叫bin存放编译后的程序。把源码和cmakelist文件丢到src目录里

![](8.png)

我这里源码只有一个main文件，如果你有很多的话也是一样的，这个cMakeLists.txt具体写什么呢？打开编辑，

    
    
    ######## A simple cmakelists.txt file for OpenCV() #############  
    cmake_minimum_required(VERSION 2.8)                          # 初始化Cmake版本检测  
    PROJECT(morphology)                                       # 工程名  
       
    FIND_PACKAGE( OpenCV REQUIRED )                              # 环境变量中寻找OpenCV的库的位置  
    INCLUDE_DIRECTORIES(${MyInpainting_SOURCE_DIR}              # 将目录下的头文件加入到工程  
    )  
      
    ADD_EXECUTABLE(morphology main.cpp)                         # 将文件加入工程，有多少.c或者cpp都加进去  
    TARGET_LINK_LIBRARIES (morphology ${OpenCV_LIBS})         # 这两行的次序也不能变!加入动态链接库  
    # ########## end ####################################  

  
这里的morphology是我的工程名，你根据你的需要更改，我这个程序是写形态学处理的所以叫这个名字，你如果叫恐龙就起名叫dinosaur。然后FIND_PACHAGE就是用来从环境变量的opencv_dir这个路径来寻找opencv的目录的，include_directories是用来添加头文件路径的，我这里用的头文件都只在这个src里面，所以就这样写了，如果有其他路径也写在这个小括号里，用空格隔开就行了。ADD_EXECUTABLE就是添加所有源文件的了，如果有其他的也是用空格隔开，不用加引号，TARGET_LINK_LIBRARIES是添加lib的静态链接库目录的，如果添加了opencv_dir的环境变量这里这样写也就ok了。最后打开cmake生成这个项目的工程吧。

![](9.png)  

依然如此，填写完源文件和生成文件路径后点Configure和Generate，不出意外的话用VS打开就好了。

打开VS看到这里有三个项目，找到我们要的morphology点右键，设为启动项，然后生成就可以了。接下来就该自行发挥写程序的能力了。

