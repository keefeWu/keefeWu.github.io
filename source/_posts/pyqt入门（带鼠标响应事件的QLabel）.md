---
title: pyqt入门（带鼠标响应事件的QLabel）
date: 2020-02-08 15:18:15
tags:
---
最近项目需要，简单的写起了一个界面，装了一个pyqt5.1，以下记录一下。

我主要使用的功能是Qlabel，它不但可以显示文字，还可以显示图片。但是它自身是没有鼠标监听功能的，所以我自己重新封装了一层。

首先是引用部分，我引用了一大堆，反正都有用，我还没有完全摸清楚，将来再补充

    
    
    #coding:utf-8
    from PyQt5.QtWidgets import QApplication, QLabel, QSlider  
    from PyQt5 import QtWidgets  
    import PyQt5.QtCore 
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QPixmap,  QImage
    from PyQt5.QtWidgets import *

以下是我封装的Qlabel类，因为带有按键监听功能，所以我改名叫button label了

    
    
    class BtnLabel(QLabel):  
        def __init__(self,parent=None):  
            super(BtnLabel,self).__init__(parent)  
            self.if_mouse_press = False
    
        def mouseMoveEvent(self,e):  
            print ('mouse move:(%d,%d)\n'%(e.pos().x(),e.pos().y()))
            if self.if_mouse_press:
                dialog.move_point(e.pos().x(),e.pos().y())
        def mousePressEvent(self,e):  
            print ('mousePressEvent(%d,%d)\n'%(e.pos().x(),e.pos().y()))
            self.if_mouse_press = True
            dialog.move_point(e.pos().x(),e.pos().y())
    
        def mouseReleaseEvent(self,e):  
            print ('mouseReleaseEvent(%d,%d)\n'%(e.pos().x(),e.pos().y()))
            self.if_mouse_press = False
    

可以看到，我加了鼠标移动，鼠标点击，鼠标释放三个功能，但是它的鼠标点击是个事件，不是一个状态，所以我要想实现鼠标拖动效果得自己添加一个状态，也就是self.if_mouse_press这个家伙，当点击的时候为true，松开的时候为false，这样就可以判断移动的时候是不是点击状态了。

然后我需要点击的时候能够干一个事情，执行这个move_point函数，但是呢这个函数在创建窗口的那个类里面，我暂时想不到什么好办法，只好把窗口对象dialog写成全局变量了，这绝对不是一个好办法，日后我找到更好的办法了会专门讲解一下的。

然后就是dialog的那个类，也就是主窗口的类

    
    
    class MainDialog(QDialog):  
        def __init__(self,parent=None):  
            super(MainDialog,self).__init__(parent)  
    self.input_label = BtnLabel(self)  
            self.input_label.setGeometry(160, 40, 640, 480)  
    
            self.output_label = BtnLabel(self)  
            self.output_label.setGeometry(900, 40, 640, 480)  
    
    
    
            #set open file button
            self.open_btn = QtWidgets.QPushButton(self)  
            self.open_btn.setObjectName("open_btn")  
            self.open_btn.setGeometry(1, 0, 100, 40)
    
            self.open_btn.setText("open")  
            self.open_btn.clicked.connect(self.open_file) 
    
            #set calculate button
            self.calc_btn = QtWidgets.QPushButton(self)  
            self.calc_btn.setObjectName("calc_btn")  
            self.calc_btn.setGeometry(1, 60, 100, 40)
            self.calc_btn.setText("calculate")  
            self.calc_btn.clicked.connect(self.calculate)  
    
            #set save file button
            self.save_btn = QtWidgets.QPushButton(self)  
            self.save_btn.setObjectName("save_btn")  
            self.save_btn.setGeometry(1, 120, 100, 40)
    
            self.save_btn.setText("save")  
            self.save_btn.clicked.connect(self.save_file)  
    
            #set add point button
            self.add_point_btn = QtWidgets.QPushButton(self)  
            self.add_point_btn.setObjectName("add_point_btn")  
            self.add_point_btn.setGeometry(1, 180, 100, 40)
    
            self.add_point_btn.setText("add point")  
            self.add_point_btn.clicked.connect(self.add_point_on_click)  
    
            #set erase point button
            self.erase_point_btn = QtWidgets.QPushButton(self)  
            self.erase_point_btn.setObjectName("erase_point_btn")  
            self.erase_point_btn.setGeometry(1, 240, 100, 40)
    
            self.erase_point_btn.setText("erase point")  
            self.erase_point_btn.clicked.connect(self.erase_point_on_click)  
    
            #set add border button
            self.add_border_btn = QtWidgets.QPushButton(self)  
            self.add_border_btn.setObjectName("add_border_btn")  
            self.add_border_btn.setGeometry(1, 300, 100, 40)
    
            self.add_border_btn.setText("add border")  
            self.add_border_btn.clicked.connect(self.add_border_on_click)  
    
            #set result text
            self.text_label = QLabel(self)
            self.text_label.setAlignment(Qt.AlignCenter)  
            self.text_label.setGeometry(1, 360, 140, 100)  

这里面我定义了两个btnlabel放图片，一个qlabel放文字，还有6个按钮，先从qlabel讲起吧。

    
    
            self.text_label = QLabel(self)
            self.text_label.setAlignment(Qt.AlignCenter)  
            self.text_label.setGeometry(1, 360, 140, 100)  

第一句就是创建一个QLabel的对象，第二句是设置里面文本的对齐方式，为居中，第三句是设置坐标以及尺寸，这个setGeometry函数有4个参数，分别是起始点也就是左上角的x,y然后是宽度和高度，那些button其实也用了这个函数

想要在里面写字的话就用self.text_label.setText('abc')这个函数，但是注意框的大小不会随着文字长度而变化，如果文字过长就会看不见了，至于具体怎么调节我以后再仔细研究。

button的创建和qlabel的创建大同小异，唯一区别的就是button有绑定点击响应的函数，

    
    
    self.add_point_btn.setObjectName("add_point_btn")  

使用这句就把add_point_btn这个按钮绑定了add_point_btn这个函数了，一旦点击这个按钮就执行这个函数，具体想干什么就随意啦。我是让他点击这个按钮可以在我的两个qlabel上面画画

![](0.png)  

我的最终效果是这样的，下一节我将讲解如何用opencv的mat显示上来

