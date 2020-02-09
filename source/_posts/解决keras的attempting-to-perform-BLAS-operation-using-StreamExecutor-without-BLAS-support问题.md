---
title: >-
  解决keras的attempting-to-perform-BLAS-operation-using-StreamExecutor-without-BLAS-support问题
date: 2020-02-08 15:17:28
tags:
---
今天跑一个keras的程序，在别人电脑上都没问题，到了我的电脑上就出错了，提示的是

    
    
    attempting to perform BLAS operation using StreamExecutor without BLAS support

其实原因就是找不到合适的gpu，要么是没找对gpu，要么就是显存不够，可以通过显示设置gpu使用  
`  
CUDA_VISIBLE_DEVICES=0  
`  
来指定使用0号gpu，如果只有一个gpu的话，使用`CUDA_VISIBLE_DEVICES=1`切换成使用cpu也可以解决。  
还有一种方法，其实才是最正规的方法，就是限制程序使用显存的大小  
`  
import tensorflow as tf  
from keras.backend.tensorflow_backend import set_session  
config = tf.ConfigProto()  
config.gpu_options.per_process_gpu_memory_fraction = 0.3  
set_session(tf.Session(config=config))  
`  
这里使用了tensorflow的config，加上这一段话，keras就会加载tensorflow的config，并且我设置的显存使用，他就只会使用30%的显存，不会多要。

