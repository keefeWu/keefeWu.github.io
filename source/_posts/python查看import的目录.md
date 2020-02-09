---
title: python查看import的目录
date: 2020-02-08 15:21:09
tags:
---
如果想查询某个指定库的目录，可以通过如下方法得到

    
    
    import a_module
    print a_module.__file__

  
例如，我想用opencv,那个库的名字叫做cv2，如果我想找到他的地址，那么我就应该这样：

    
    
    import cv2
    print cv2.__file__

  
可以看到执行这两行就会打印出地址：

/usr/local/lib/python2.7/dist-packages/cv2/cv2.so  

  

如果想查询import的包含目录，那么可以通过

    
    
    import os
    print os.path

  
这个方法打印所有引用的目录

