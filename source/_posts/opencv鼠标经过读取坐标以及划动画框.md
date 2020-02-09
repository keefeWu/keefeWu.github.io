---
title: opencv鼠标经过读取坐标以及划动画框
date: 2020-02-08 15:21:02
tags:
---
本代码copy自：

http://www.cnblogs.com/lidabo/p/3437587.html  

    
    
    #include <opencv2/opencv.hpp>
    #include <opencv2/core/core.hpp>  
    #include <opencv2/highgui/highgui.hpp>  
    #include <stdio.h>  
      
    using namespace cv;  
      
    cv::Mat org,dst,img,tmp;  
    //event:id of mouse click event;
    //x,y:mouse coordinate;;
    //flags: drag or keyboard id  
    void on_mouse(int event,int x,int y,int flags,void *ustc)
    {  
        static Point pre_pt = Point(-1,-1);//pre coordinate;  
        static Point cur_pt = Point(-1,-1);//current coordinate;  
        char temp[16];  
        if (event == CV_EVENT_LBUTTONDOWN)//left click，draw a circle on the position  
        {  
            org.copyTo(img);
            sprintf(temp,"(%d,%d)",x,y);  
            pre_pt = Point(x,y);  
            putText(img,temp,pre_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255),1,8);//show the coordinate  
            circle(img,pre_pt,2,Scalar(255,0,0,0),CV_FILLED,CV_AA,0);//draw a circle  
            imshow("img",img);  
        }  
        else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))//mouse pass by without click  
        {  
            img.copyTo(tmp);  
            sprintf(temp,"(%d,%d)",x,y);  
            cur_pt = Point(x,y);  
            putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255));//real time show coordinate  
            imshow("img",tmp);  
        }  
        else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))//left click and move mouse,draw a real time rectangle  
        {  
            img.copyTo(tmp);  
            sprintf(temp,"(%d,%d)",x,y);  
            cur_pt = Point(x,y);  
            putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255));  
            rectangle(tmp,pre_pt,cur_pt,Scalar(0,255,0,0),1,8,0);  
            imshow("img",tmp);  
        }  
        else if (event == CV_EVENT_LBUTTONUP)//release left click,draw a rectangle  
        {  
            org.copyTo(img);  
            sprintf(temp,"(%d,%d)",x,y);  
            cur_pt = Point(x,y);  
            putText(img,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255));  
            circle(img,pre_pt,2,Scalar(255,0,0,0),CV_FILLED,CV_AA,0);  
            rectangle(img,pre_pt,cur_pt,Scalar(0,255,0,0),1,8,0);
            imshow("img",img);  
            img.copyTo(tmp);  
            //show the roi that cut by rectangle 
            int width = abs(pre_pt.x - cur_pt.x);  
            int height = abs(pre_pt.y - cur_pt.y);  
            if (width == 0 || height == 0)  
            {  
                printf("width == 0 || height == 0");  
                return;  
            }  
            dst = org(Rect(min(cur_pt.x,pre_pt.x),min(cur_pt.y,pre_pt.y),width,height));  
            namedWindow("dst");  
            imshow("dst",dst);  
            waitKey(0);  
        }  
    }  
    int main()  
    {  
        org = imread("/home/bot/Downloads/loading_state/data_0_1504602551362.jpg");  
        org.copyTo(img);  
        org.copyTo(tmp);  
        namedWindow("img"); 
        setMouseCallback("img",on_mouse,0);//callback  
        imshow("img",img);  
        cv::waitKey(0);  
        return 0;
    }  

  
  

  

