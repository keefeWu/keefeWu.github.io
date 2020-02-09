---
title: pyqt入门（opencv与QLabel）
date: 2020-02-08 15:18:11
tags:
---
上一讲讲解了如何绘制一个QLabel，这一讲来讲解如何用QLabel显示opencv的图片

先直接上代码

    
    
            rgb_img = cv2.cvtColor(opencv_img, cv2.COLOR_BGR2RGB)                                          
            QImg = QImage(rgb_img.data, self.width, self.height, self.bytesPerLine,QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(QImg)
            self.output_label.setPixmap(pixmap)

假设点击了某个按钮，计算出了一个opencv的图片opencv_img

首先QLabel的图片使用的是QPixmap这个类型的图，那么我们就要将opencv转成这个类型

使用第二句和第三句就可以了，第一句是因为QPixmap需要的是RGB图，而opencv的是BGR的图。正好是反的，所以就要转一道。

QImage这个函数需要五个参数，第一个是图片的数据，直接把转通道后的data给他就可以了，第二个参数和第三个参数分别是图片的宽和高，第四个参数是每一行的比特数，8位三通道的就是宽度*3，也就是传入一个self.width*3就可以了，第五个参数是数据类型了，照我这个给就可以了

然后把生成的QImg转成pixmap

对QLabel对象使用setPixmap这个函数就可以了，参数就是pixmap这个图，这句一执行就更新了QLabel那个框那里显示的图片了，再次附上效果图

![](0.png)  

