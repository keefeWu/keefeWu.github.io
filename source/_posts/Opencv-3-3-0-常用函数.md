---
title: Opencv-3-3-0-常用函数
date: 2020-02-08 15:18:50
tags:
---
## 如何调图像的亮度和对比度？

    
    
    转载自： https://www.cnblogs.com/cheungxiongwei/p/7577013.html//如何增加图片的对比度或亮度？
    void contrastOrBrightAdjust(InputArray &src,OutputArray &dst,int contrastValue,int brightValue)
    {
      cv::Mat _dst = cv::Mat::zeros( src.size(), src.type() );
      for(int y = 0; y < src.rows; y++ )
      {
        for(int x = 0; x < src.cols; x++ )
        {
          for(int c = 0; c < 3; c++ )
          {
            _dst.at<cv::Vec3b>(y,x)[c]= cv::saturate_cast<uchar>( (contrastValue*0.01)*(src.at<cv::Vec3b>(y,x)[c] ) + brightValue);
          }
        }
      }
      dst = _dst;
      return;
    }row

## 如何在 `cv::imshow()` 函数上创建滑块 `cv::createTrackbar()`？

    
    
    //void callBack(int, void*);
    //getTrackbarPos()//获取当前滑块位置
    cv::Mat src = cv::Mat::zeros(cv::Size(888,888),3);
    cv::nameWindow("Win7",cv::WINDOW_NORMAL);
    CV::createTrackbar("Trackbar","Win7",0,100,NULL);
    cv::imshow("Win7",src);

## 如何读取图像和保存图像?

    
    
    cv::Mat src = imread("C:/Desktop.jpg");
    imWrite("C:/Desktops.jpg",src(Rect(0,0,88,88)));

## 轮廓检测 `cv::Canny()`

    
    
    cv::Mat src = imread("C:/Desktop.jpg");
    cvtColor(src,src,CV_BGR2GRAY);
    Canny(src,src,120,180);
    threshold(src,src,0,255,CV_THRESH_OTSU);
    cv::nameWindow("Win7",cv::WINDOW_NORMAL);
    cv::imshow("Win7",src);
    //findContours()//获取轮廓个数
    //DrawContours()//画轮廓
    //arcLength()//弧长
    //approxPolyDP()//轮廓近似
    //minAreaRect()//计算最小面积的外接矩形
    //contourArea()//计算轮廓内连通区域的面积
    //pointPolygenTest()//判断一个点是否在一个多边形内
    //mathShapes()//比较两个形状的相似性
    //dilate()//膨胀
    //resize()//图像的放大和缩小
    //GaussianBlur()//高斯滤波
    //medianBlur()//中值滤波
    //blur()//均值滤波
    //bilateralFilter()//双边滤波
    //boxFilter()//方框滤波
    //split()//分离通道

## 霍夫直线检测

    
    
    HoughLins();
    HoughLinsP();
    
    
    //俗话说：好记性不如烂笔头
    //用到opencv 中的函数时往往会一时记不起这个函数的具体参数怎么设置，故在此将常用函数做一汇总；
    
    Mat srcImage = imread("C:/Users/Administrator/Desktop/车牌识别/车牌图像库/1.jpg");//读入图像函数
    
    imshow("原图",srcImage);//显示图像函数
    
    imwrite("图3.jpg",imageRIO);//保存图像函数
    
    Mat imageRIO = srcImage(Rect(50,50,100,000));//感兴趣区域函数
    
    cvtColor(srcImage,dstImage,CV_BGR2GRAY);//图像灰度化
    
    
    //边缘检测 Sobel Laplacian Canny 其中Canny算子只能处理（8位）灰度图，其余两种8位32位都可以
    Mat grad_x,grad_y;
    Sobel(imgGray,grad_x,CV_8U,1,0,3,1,1);//X方向上的Sobel算子检测，其中3，1，0都是默认值
    
    Sobel(imgGray,grad_y,CV_8U,0,1,3,1,0);//Y方向上的Sobel算子检测，其中3，1，0都是默认值
    
    addWeighted(grad_x,0.5,grad_y,0.5,0,dstImage);//合并梯度
    
    Laplacian(imgGray,dstImage,CV_8U);
    
    Canny(imgGray,dstImage,50,200,3);//50和200表示第一个滞后性阈值和第二个滞后性阈值，较小者用于边缘连接，较大者控制强边缘的初始段，达阈值opnecv推荐为小阈值的3倍；
    //3表示应用的Sobel算子的孔径大小 有默认值为3；
    
    // 寻找轮廓 只处理8位 即灰度图像
    vector<vector<Point>> contours;
    findContours(imgGray,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    drawContours(dstImage,contours,-1,Scalar(0),3);
    imshow("轮廓图",dstImage);
    
    
    //阈值化操作
    threshold(srcImage,dstImage,100,255,3);
    imshow("固定阈值化图像",dstImage);
    adaptiveThreshold(imgGray,dstImage,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,3,1);
    imshow("自适应阈值化图像",dstImage);
    
    
    // resize函数实现
    resize(srcImage,dstImage,Size(),0.5,0.5);//缩小为一半
    resize(srcImage,dstImage,Size(),2,2);//放大2倍
    resize(srcImage,dstImage,Size(srcImage.cols*3,srcImage.rows*3));//放大3倍
    
    
    // 金字塔函数实现
    pyrUp(srcImage,dstImage,Size(srcImage.cols*2,srcImage.rows*2));// 放大2倍
    pyrDown(srcImage,dstImage,Size(srcImage.cols/2,srcImage.rows/2));// 缩小2倍
    
    //漫水填充算法
    Rect ccomp;
    floodFill(srcImage,Point(50,300),Scalar(155,255,55),&ccomp,Scalar(20,20,20),Scalar(20,20,20));
    
    //膨胀腐蚀
    Mat element = getStructuringElement(MORPH_RECT,Size(15,15));
    erode(srcImage,dstImage,element);//腐蚀函数
    dilate(srcImage,dstImage,element);//膨胀函数
    
    morphologyEx(g_bgrImage,g_bgrImage, MORPH_CLOSE, element);//闭运算
    
    morphologyEx(g_bgrImage,g_bgrImage, MORPH_OPEN, element);//开运算
    
    
    //滤波
    boxFilter(srcImage,dstImage,-1,Size(3,3));
    imshow("方框滤波图",dstImage);
    
    blur(srcImage,dstImage,Size(3,3));
    imshow("均值滤波图",dstImage);
    
    GaussianBlur(srcImage,dstImage,Size(5,7),1,1);
    imshow("高斯滤波图",dstImage);
    
    medianBlur(image,out,7);//中值滤波，7为孔径的线性尺寸
    bilateralFilter(src,dst,d,sigmaColor,sigmaSpace);//双边滤波，d表示过滤过程中每个像素邻域的直径，sigmaColor颜色空间滤波器的sigma值，sigmaSpace表示坐标空间中滤波器的sigma值
    bilateralFilter(image,out,25,25*2,25/2);
    
    //《未完待续》
    
    
    关于“轮廓检测”和“边缘检测”这两个自己也弄的不是特别清楚，可能确实比较相似吧。下面简单说一下自己的看法。
    区别：
    边缘检测主要是通过一些手段检测数字图像中明暗变化剧烈（即梯度变化比较大）像素点，偏向于图像中像素点的变化。
    如canny边缘检测，结果通常保存在和源图片一样尺寸和类型的边缘图中。
    轮廓检测指检测图像中的对象边界，更偏向于关注上层语义对象。
    如OpenCV中的findContours()函数， 它会得到每一个轮廓并以点向量方式存储，除此也得到一个图像的拓扑信息，即一个轮廓的后一个轮廓、前一个轮廓、父轮廓和内嵌轮廓的索引编号。
    联系：
    我们在做图像的轮廓检测时通常可以先检测边缘，再将检测到的边缘进行进一步处理，得到图像的轮廓。

转载请注明出处并保持作品的完整性,谢谢

转载自： <https://www.cnblogs.com/cheungxiongwei/p/7577013.html>

