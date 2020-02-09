java与c、c++通信中有一个最常用的方法，就是JNI的方式。首先让c++生成动态链接库，然后用java来调用。下面重点演示linux下的做法。

首先用eclipse建立一个java类：

![]()

类名就叫JNIDemo

接着写上如下代码：

![]()

    
    
    package com.jni.demo;  
    public class JNIDemo {  
        //定义一个本地方法  
        public native String entrance(String P1,String P2,String P3);  
        public static void main(String[] args){  
            //调用动态链接库  
            System.loadLibrary("faceDetect");  
            JNIDemo jniDemo = new JNIDemo();  
            jniDemo.entrance("paras1",
            		"
    
    
    paras2

", "paras2"); } }

  

我写的是一个测试的代码，入口函数是entrance，所以再main函数前声明要调用的方法。一共三个三个参数，paras1,paras2,paras3。三个都是string类型的。

有了这个声明以后，接下来找到这个项目的目录，用命令行进入bin目录，然后执行javah生成头文件。

![]()  

执行成功之后就可以看到bin目录下出现了一个![]()头文件。

把这个头文件添加到要写的c++工程目录里，找到定义的入口函数，

![]()  

    
    
    JNIEXPORT jstring JNICALL Java_com_jni_demo_JNIDemo_entrance
      (JNIEnv *, jobject, jstring, jstring, jstring);

  

这个就是我们的入口函数的声明，在c++项目中引入这个头文件，然后把这个函数作为入口函数，不过声明时候几个变量只有类型，没有变量，所以再cpp文件中要加上变量。

![]()  

不过jstring类型不能直接作为c++的string使用，需要通过函数转换成char*类型。转换方式是：

    
    
    const char* str;
    str = env->GetStringUTFChars(datPath, false); 

  

同时c++的string和char*也不直接返回，也是需要通过函数转换成jstring类型：

    
    
    char* result = "error";
        	jstring rtstr = env->NewStringUTF(result);

再看env这个对象，这个对象就是java与c++沟通的一个桥梁，第二个参数暂时不清楚。后面三个是我们函数自己定义的参数。

实现好了以后编译成.so文件。Cmake的方法是

ADD_LIBRARY(faceDetect SHARED faceDetect.cpp)

通过这个方法生成动态链接库

把动态链接库地址添加到eclipse的java目录里。

在properties里找到这个

![]()

JRE下面的Native library

点击Edit，添加so的目录。保存运行。如果不出意外就会执行我们写的C++函数了。

如果不是用的eclipse的话，在命令行添加环境变量，export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:自己的库路径

这个命令只在shell结束前有效。如果想要长期有效就需要在环境变量里添加了，也就是vi /etc/profile

然后在最下面添加这句话。

  

注意javah生成的头文件是会随着目录结构而改变的，如果你要做代码移植，并且目录结构变了，记得重新生成一遍头文件，不然会提示找不到函数的。错误如下

java.lang.UnsatisfiedLinkError:库名.方法名()  

这个时候不要慌，一定是方法名有不一样的地方，记得重新生成头文件然后在cpp文件里改过来就行了。

  
  

