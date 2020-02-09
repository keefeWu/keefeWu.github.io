---
title: opencv-matchTemplate函数的解析
date: 2020-02-08 15:20:17
tags:
---
opencv有个MatchTemplate_Demo.cpp文件实现了matchTemplate这个函数的调用demo。

首先这个函数的功能是根据一个小块物体的图片，然后在整幅图里面搜索和他最像的区域。

算法在他的官方文档上讲的很清楚了，一共有六种方法：

  1. **method=CV_TM_SQDIFF**

![R\(x,y\)= \\sum _{x',y'} \(T\(x',y'\)-I\(x+x',y+y'\)\)^2](0.png)

  2. **method=CV_TM_SQDIFF_NORMED**

![R\(x,y\)= \\frac{\\sum_{x',y'}
\(T\(x',y'\)-I\(x+x',y+y'\)\)^2}{\\sqrt{\\sum_{x',y'}T\(x',y'\)^2 \\cdot
\\sum_{x',y'} I\(x+x',y+y'\)^2}}](1.png)

  3. **method=CV_TM_CCORR**

![R\(x,y\)= \\sum _{x',y'} \(T\(x',y'\)  \\cdot I\(x+x',y+y'\)\)](2.png)

  4. **method=CV_TM_CCORR_NORMED**

![R\(x,y\)= \\frac{\\sum_{x',y'} \(T\(x',y'\) \\cdot
I\(x+x',y+y'\)\)}{\\sqrt{\\sum_{x',y'}T\(x',y'\)^2 \\cdot \\sum_{x',y'}
I\(x+x',y+y'\)^2}}](3.png)

  5. **method=CV_TM_CCOEFF**

![R\(x,y\)= \\sum _{x',y'} \(T'\(x',y'\)  \\cdot I\(x+x',y+y'\)\)](4.png)

where

![\\begin{array}{l} T'\(x',y'\)=T\(x',y'\) - 1/\(w  \\cdot h\)  \\cdot \\sum
_{x'',y''} T\(x'',y''\) \\\\ I'\(x+x',y+y'\)=I\(x+x',y+y'\) - 1/\(w  \\cdot
h\)  \\cdot \\sum _{x'',y''} I\(x+x'',y+y''\) \\end{array}](5.png)

  6. **method=CV_TM_CCOEFF_NORMED**

![R\(x,y\)= \\frac{ \\sum_{x',y'} \(T'\(x',y'\) \\cdot I'\(x+x',y+y'\)\) }{
\\sqrt{\\sum_{x',y'}T'\(x',y'\)^2 \\cdot \\sum_{x',y'} I'\(x+x',y+y'\)^2}
}](6.png)

  

a最明显，就是遍历每个像素，对于单个像素，像其有下角画框，框的大小就是要找的样例的大小，我们称之为template。对于框内的每个像素值和template的像素值逐点作差然后平方求和，类似一个卷积的过程。所以最终自然是越小越像咯，所以在matchTemplate这个函数的result图像中最小的那个点就是最像的区域。对于之后的五种方法类似，x'就是template的坐标，实际上也就是一个浮动位。

所以最终最像的结果依次是

a:最小值 b:最小值 c:最大值 d:最大值 e:最大值 f:最大值

  

再看demo的代码：

    
    
    #include "opencv2/highgui/highgui.hpp"
    #include "opencv2/imgproc/imgproc.hpp"
    #include <iostream>
    #include <stdio.h>
    
    using namespace std;
    using namespace cv;
    
    /// Global Variables
    Mat img; Mat templ; Mat result;
    char* image_window = "Source Image";
    char* result_window = "Result window";
    
    int match_method;
    int max_Trackbar = 5;
    
    /// Function Headers
    void MatchingMethod( int, void* );
    
    /** @function main */
    int main( int argc, char** argv )
    {
      /// Load image and template
      img = imread( argv[1], 1 );
      templ = imread( argv[2], 1 );
    
      /// Create windows
      namedWindow( image_window, CV_WINDOW_AUTOSIZE );
      namedWindow( result_window, CV_WINDOW_AUTOSIZE );
    
      /// Create Trackbar
      char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
      createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );
    
      MatchingMethod( 0, 0 );
    
      waitKey(0);
      return 0;
    }
    
    /**
     * @function MatchingMethod
     * @brief Trackbar callback
     */
    void MatchingMethod( int, void* )
    {
      /// Source image to display
      Mat img_display;
      img.copyTo( img_display );
    
      /// Create the result matrix
      int result_cols =  img.cols - templ.cols + 1;
      int result_rows = img.rows - templ.rows + 1;
    
      result.create( result_rows, result_cols, CV_32FC1 );
    
      /// Do the Matching and Normalize
      matchTemplate( img, templ, result, match_method );
      normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
    
      /// Localizing the best match with minMaxLoc
      double minVal; double maxVal; Point minLoc; Point maxLoc;
      Point matchLoc;
    
      minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    
      /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
      if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
        { matchLoc = minLoc; }
      else
        { matchLoc = maxLoc; }
    
      /// Show me what you got
      rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
      rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
    
      imshow( image_window, img_display );
      imshow( result_window, result );
    
      return;
    }

  
其中承载的有3个Mat ，img是大图，templ是要找的小块，就是要在img上找和templ最像的区域。result就是返回计算结果，他的大小是img-
templ，因为img上每个点找templ的结果，所以最右和最下的像素到边界的距离小于templ的大小了，所以就不找了。所以result上的每个点的像素值就是相似度了，具体是越大越像还是越小越像参照上面的解说。

所以demo用了

    
    
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    
      /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
      if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
        { matchLoc = minLoc; }
      else
        { matchLoc = maxLoc; }
    

  

来找最像的那个区域左上角的点

然后往下画矩形就是最像的那个区域了。

    
    
    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
      rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

  

