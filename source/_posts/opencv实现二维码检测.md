---
title: opencv实现二维码检测
date: 2020-07-03 14:03:44
tags: [opencv]
categories: 
- 图像处理
---
# opencv调用
opencv从4代之后推出了二维码识别接口.调用方法是这样的.
```
import cv2
img = cv2.imread('data/qrcode.jpg')
img = cv2.imread('qrcode/11.jpg')
qrcode = cv2.QRCodeDetector()
result, points, code = qrcode.detectAndDecode(img)
```
返回值有三个,第一个<code>result</code>就是解码后的内容,例如我这个二维码的结果是"https://space.bilibili.com/37270391",当然也可以是个纯数字.第二个<code>points</code>是二维码轮廓的四个角,从左上角顺时针转的.第三个<code>code</code>是二维码的原始排列,也就是每个点是0还是255的一个矩阵.白色是255,黑色是0.
调用起来十分方便,而且如果不需要解码,只是想定位的话可以调用<code>detect</code>函数,返回结果就只有四个角点了.

# opencv源码
## 定位
它的源码放在"modules/objdetect/src/qrcode.cpp"文件里面,很容易就能找到.
### detect
直接看<code>detect</code>函数
```
bool QRCodeDetector::detect(InputArray in, OutputArray points) const
{
    Mat inarr;
    if (!checkQRInputImage(in, inarr))
        return false;

    QRDetect qrdet;
    qrdet.init(inarr, p->epsX, p->epsY);
    if (!qrdet.localization()) { return false; }
    if (!qrdet.computeTransformationPoints()) { return false; }
    vector<Point2f> pnts2f = qrdet.getTransformationPoints();
    updatePointsResult(points, pnts2f);
    return true;
}
```
它第一步调用的是<code>QRDetect</code>类的<code>localization</code>函数
然后我们再看这个<code>localization</code>函数

### localization
```
    vector<Vec3d> list_lines_x = searchHorizontalLines();
    if( list_lines_x.empty() ) { return false; }
    vector<Point2f> list_lines_y = separateVerticalLines(list_lines_x);
    if( list_lines_y.empty() ) { return false; }

    vector<Point2f> centers;
    Mat labels;
    kmeans(list_lines_y, 3, labels,
           TermCriteria( TermCriteria::EPS + TermCriteria::COUNT, 10, 0.1),
           3, KMEANS_PP_CENTERS, localization_points);
    fixationPoints(localization_points);
```
我提取了一些关键部分,它首先调用了<code>searchHorizontalLines</code>这个函数,用来寻找符合角点轮廓要求的水平线段.然后利用这些线段,调用<code>separateVerticalLines</code>寻找垂直的线段.

### searchHorizontalLines

```
    CV_TRACE_FUNCTION();
    vector<Vec3d> result;
    const int height_bin_barcode = bin_barcode.rows;
    const int width_bin_barcode  = bin_barcode.cols;
    const size_t test_lines_size = 5;
    double test_lines[test_lines_size];
    vector<size_t> pixels_position;

    for (int y = 0; y < height_bin_barcode; y++)
```
这个函数先按行去找横线,所以这里是y从0到<code>height_bin_barcode</code>
```
        pixels_position.clear();
        const uint8_t *bin_barcode_row = bin_barcode.ptr<uint8_t>(y);

        int pos = 0;
        for (; pos < width_bin_barcode; pos++) { if (bin_barcode_row[pos] == 0) break; }
        if (pos == width_bin_barcode) { continue; }

        pixels_position.push_back(pos);
        pixels_position.push_back(pos);
        pixels_position.push_back(pos);
```
然后初始化了接下来要用的<code>pixels_position</code>
接下来就是重点了,从左到右一个点一个点的判断
```
        uint8_t future_pixel = 255;
        for (int x = pos; x < width_bin_barcode; x++)
        {
            if (bin_barcode_row[x] == future_pixel)
            {
                future_pixel = static_cast<uint8_t>(~future_pixel);
                pixels_position.push_back(x);
            }
        }
```
首先设定下一个待搜索的像素是白色(255),然后从左到右找,一直到白色了就把这个位置记录下来,作为白线的起点,然后吧下一个待寻找的点改成黑色<code>future_pixel = static_cast<uint8_t>(~future_pixel);</code>,这样循环下来就能找到这一行所有的线段了.
然后判断一下这些线段长度是不是满足二维码角点轮廓的比例
```
        pixels_position.push_back(width_bin_barcode - 1);
        for (size_t i = 2; i < pixels_position.size() - 4; i+=2)
        {
            test_lines[0] = static_cast<double>(pixels_position[i - 1] - pixels_position[i - 2]);
            test_lines[1] = static_cast<double>(pixels_position[i    ] - pixels_position[i - 1]);
            test_lines[2] = static_cast<double>(pixels_position[i + 1] - pixels_position[i    ]);
            test_lines[3] = static_cast<double>(pixels_position[i + 2] - pixels_position[i + 1]);
            test_lines[4] = static_cast<double>(pixels_position[i + 3] - pixels_position[i + 2]);

            double length = 0.0, weight = 0.0;  // TODO avoid 'double' calculations

            for (size_t j = 0; j < test_lines_size; j++) { length += test_lines[j]; }

            if (length == 0) { continue; }
            for (size_t j = 0; j < test_lines_size; j++)
            {
                if (j != 2) { weight += fabs((test_lines[j] / length) - 1.0/7.0); }
                else        { weight += fabs((test_lines[j] / length) - 3.0/7.0); }
            }

            if (weight < eps_vertical)
            {
                Vec3d line;
                line[0] = static_cast<double>(pixels_position[i - 2]);
                line[1] = y;
                line[2] = length;
                result.push_back(line);
            }
        }
```
这里这个<code>pixels_position</code>为什么敢直接取到4呢,我们在回到上面看
```
        int pos = 0;
        for (; pos < width_bin_barcode; pos++) { if (bin_barcode_row[pos] == 0) break; }
        if (pos == width_bin_barcode) { continue; }

        pixels_position.push_back(pos);
        pixels_position.push_back(pos);
        pixels_position.push_back(pos);
```
他这里初始化的时候就直接推进去了3个0,最后又把右边界推进去了<code>pixels_position.push_back(width_bin_barcode - 1);</code>
所以他就可以从左往右把连续的5个线段拿出来判断了,滑动的方法就和一维的卷积一样,步长是1,宽度是5.
```
            test_lines[0] = static_cast<double>(pixels_position[i - 1] - pixels_position[i - 2]);
            test_lines[1] = static_cast<double>(pixels_position[i    ] - pixels_position[i - 1]);
            test_lines[2] = static_cast<double>(pixels_position[i + 1] - pixels_position[i    ]);
            test_lines[3] = static_cast<double>(pixels_position[i + 2] - pixels_position[i + 1]);
            test_lines[4] = static_cast<double>(pixels_position[i + 3] - pixels_position[i + 2]);
```
接下来就是判断这5条线的长度比例是不是满足,比例如下图
![角点](https://blog.357573.com/2020/07/03/opencv实现二维码检测/corner.png)  
看这个图就很明显了,如果是中间位置的话,这个比例应该是1:1:3:1:1这5条线段,那么除了第3条,其他都是1,总长度为7,所以我们按照1/7,1/7,3/7,1/7,1/7这个比例来搜索,就是下面这个代码
```
            double length = 0.0, weight = 0.0;  // TODO avoid 'double' calculations

            for (size_t j = 0; j < test_lines_size; j++) { length += test_lines[j]; }

            if (length == 0) { continue; }
            for (size_t j = 0; j < test_lines_size; j++)
            {
                if (j != 2) { weight += fabs((test_lines[j] / length) - 1.0/7.0); }
                else        { weight += fabs((test_lines[j] / length) - 3.0/7.0); }
            }

            if (weight < eps_vertical)
            {
                Vec3d line;
                line[0] = static_cast<double>(pixels_position[i - 2]);
                line[1] = y;
                line[2] = length;
                result.push_back(line);
            }
```
 这里就是我说的,判断了一下j是不是2,2的话就代表是中间那根,就只有它是3/7,其他都是1/7,然后用这个weight去+这个数是什么意思呢,我们可以看到,weight初始化的时候是0,然后每次加的是这个比例和1/7或者3/7的差,也就是加上了和我们期望想要的值的差,所以这个weight记录的就是误差和,那么我们最后判断一下累积的这个误差weight和我们设置的误差阈值<code>eps_vertical</code>比较,如果实际误差小于我们这个精度要求,那么就把这一段给记录下来,从起点的x,起点的y,到总长度都记录在了一个<code>Vec3d</code>类型的变量里面,然后添加到<code>result</code>这个数组里了.
 这样,我们这个<code>searchHorizontalLines</code>函数就收集了所有满足这个比例要求的横线序列.
 然后我们从这些横线里面找竖直方向也是这样的,把他们挑出来作为二维码角点的候选.
 那就是调用<code>separateVerticalLines</code>这个函数了

 ### separateVerticalLines
 ```
 vector<Point2f> QRDetect::separateVerticalLines(const vector<Vec3d> &list_lines)
{
    CV_TRACE_FUNCTION();

    for (int coeff_epsilon = 1; coeff_epsilon < 10; coeff_epsilon++)
    {
        vector<Point2f> point2f_result = extractVerticalLines(list_lines, eps_horizontal * coeff_epsilon);
        if (!point2f_result.empty())
        {
            vector<Point2f> centers;
            Mat labels;
            double compactness = kmeans(
                    point2f_result, 3, labels,
                    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 0.1),
                    3, KMEANS_PP_CENTERS, centers);
            if (compactness == 0)
                continue;
            if (compactness > 0)
            {
                return point2f_result;
            }
        }
    }
    return vector<Point2f>();  // nothing
}
```
 这个函数就有意思了,先调用<code>extractVerticalLines</code>函数找到所有竖着符合要求的线,直接算出了横竖交叉的中心点,然后把这些中心点聚类成了3类,但是聚类结果没有用,只是判断了一下聚类成功了没有.

核心在<code>extractVerticalLines</code>这个函数里,我们直接去这里看

### extractVerticalLines
这个函数直接取了横线的中点,也就是想从正中心开始找
```
        const int x = cvRound(list_lines[pnt][0] + list_lines[pnt][2] * 0.5);
        const int y = cvRound(list_lines[pnt][1]);
```
如果从正中心出发,那么就是分为了上下两段,我们先从上往下找
```
        test_lines.clear();
        uint8_t future_pixel_up = 255;

        int temp_length_up = 0;
        for (int j = y; j < bin_barcode.rows - 1; j++)
        {
            uint8_t next_pixel = bin_barcode.ptr<uint8_t>(j + 1)[x];
            temp_length_up++;
            if (next_pixel == future_pixel_up)
            {
                future_pixel_up = static_cast<uint8_t>(~future_pixel_up);
                test_lines.push_back(temp_length_up);
                temp_length_up = 0;
                if (test_lines.size() == 3)
                    break;
            }
        }
```
方法也是一样的,找到3段连续的截止,然后从下往上找
```
        // --------------- Search vertical down-lines --------------- //

        int temp_length_down = 0;
        uint8_t future_pixel_down = 255;
        for (int j = y; j >= 1; j--)
        {
            uint8_t next_pixel = bin_barcode.ptr<uint8_t>(j - 1)[x];
            temp_length_down++;
            if (next_pixel == future_pixel_down)
            {
                future_pixel_down = static_cast<uint8_t>(~future_pixel_down);
                test_lines.push_back(temp_length_down);
                temp_length_down = 0;
                if (test_lines.size() == 6)
                    break;
            }
        }
```
再找3段,加起来就是6段了就截止.
然后也是判断是不是符合那个比例
```

        // --------------- Compute vertical lines --------------- //

        if (test_lines.size() == 6)
        {
            double length = 0.0, weight = 0.0;  // TODO avoid 'double' calculations

            for (size_t i = 0; i < test_lines.size(); i++)
                length += test_lines[i];

            CV_Assert(length > 0);
            for (size_t i = 0; i < test_lines.size(); i++)
            {
                if (i % 3 != 0)
                {
                    weight += fabs((test_lines[i] / length) - 1.0/ 7.0);
                }
                else
                {
                    weight += fabs((test_lines[i] / length) - 3.0/14.0);
                }
            }

            if (weight < eps)
            {
                result.push_back(list_lines[pnt]);
            }
        }
    }

```
因为这次是从中间向上下两边发展,所以中间那一段就不是3/7了,而应该是两个3/14,因为是从中间出发的嘛,原本一条线被分成了两截.
最后把符合要求的中点都保存下来返回出去
```
    vector<Point2f> point2f_result;
    if (result.size() > 2)
    {
        for (size_t i = 0; i < result.size(); i++)
        {
            point2f_result.push_back(
                  Point2f(static_cast<float>(result[i][0] + result[i][2] * 0.5),
                          static_cast<float>(result[i][1])));
        }
    }
```

然后我们再回到<code>localization</code>函数,接着往下看
```
    vector<Point2f> list_lines_y = separateVerticalLines(list_lines_x);
    if( list_lines_y.empty() ) { return false; }

    vector<Point2f> centers;
    Mat labels;
    kmeans(list_lines_y, 3, labels,
           TermCriteria( TermCriteria::EPS + TermCriteria::COUNT, 10, 0.1),
           3, KMEANS_PP_CENTERS, localization_points);

    fixationPoints(localization_points);

```
他在<code>separateVerticalLines</code>这个函数找到了所有疑似中点的点,然后还聚类判断了一下,把那些聚类后的方差大于一定值的点保存下来,我不是很明白为什么要这样,可能是担心方差太小了说明点少了,误差较大吧还是什么.反正到这边又聚类了一次,数据是挑选的那些方差大的点,最终找出3个中心.然后调了<code>fixationPoints</code>找到旋转角度.

### fixationPoints
这个函数先计算了三个点两两之间的距离
```
    CV_TRACE_FUNCTION();
    double cos_angles[3], norm_triangl[3];

    norm_triangl[0] = norm(local_point[1] - local_point[2]);
    norm_triangl[1] = norm(local_point[0] - local_point[2]);
    norm_triangl[2] = norm(local_point[1] - local_point[0]);
```
然后算了一下三个夹角的余弦值
```
    cos_angles[0] = (norm_triangl[1] * norm_triangl[1] + norm_triangl[2] * norm_triangl[2]
                  -  norm_triangl[0] * norm_triangl[0]) / (2 * norm_triangl[1] * norm_triangl[2]);
    cos_angles[1] = (norm_triangl[0] * norm_triangl[0] + norm_triangl[2] * norm_triangl[2]
                  -  norm_triangl[1] * norm_triangl[1]) / (2 * norm_triangl[0] * norm_triangl[2]);
    cos_angles[2] = (norm_triangl[0] * norm_triangl[0] + norm_triangl[1] * norm_triangl[1]
                  -  norm_triangl[2] * norm_triangl[2]) / (2 * norm_triangl[0] * norm_triangl[1]);
```
如果出现了余弦大于0.85的就直接不考虑了,我算了一下,余弦0.85差不多就是30度,也就是说要是有夹角小于30度的说明找错点了,直接就返回了

```
    const double angle_barrier = 0.85;
    if (fabs(cos_angles[0]) > angle_barrier || fabs(cos_angles[1]) > angle_barrier || fabs(cos_angles[2]) > angle_barrier)
    {
        local_point.clear();
        return;
    }
```
接下来根据夹角确定了三个点的方位
```

    size_t i_min_cos =
       (cos_angles[0] < cos_angles[1] && cos_angles[0] < cos_angles[2]) ? 0 :
       (cos_angles[1] < cos_angles[0] && cos_angles[1] < cos_angles[2]) ? 1 : 2;
```

之后做了更细的一些筛选,主要是剔除了一些不该有的东西.保证挑选出来的二维码一定是准确的.

## 识别
opencv的识别用的是第三方的接口,用的<code>quirc</code>这个库,他放在"3rdparty/quirc"这个目录里了
可以看一下opencv的这个<code>decode</code>函数
```
std::string QRCodeDetector::decode(InputArray in, InputArray points,
                                   OutputArray straight_qrcode)
{
    Mat inarr;
    if (!checkQRInputImage(in, inarr))
        return std::string();

    vector<Point2f> src_points;
    points.copyTo(src_points);
    CV_Assert(src_points.size() == 4);
    CV_CheckGT(contourArea(src_points), 0.0, "Invalid QR code source points");

    QRDecode qrdec;
    qrdec.init(inarr, src_points);
    bool ok = qrdec.fullDecodingProcess();

    std::string decoded_info = qrdec.getDecodeInformation();

    if (ok && straight_qrcode.needed())
    {
        qrdec.getStraightBarcode().convertTo(straight_qrcode,
                                             straight_qrcode.fixedType() ?
                                             straight_qrcode.type() : CV_32FC2);
    }

    return ok ? decoded_info : std::string();
}
```
其实就是调用了<code>qrdec.getStraightBarcode()</code>函数,返回识别结果

## 总结
opencv这个检测,主要是利用竖直方向和水平方向线段长度比例来确定的,但这样一点旋转不变形都没有做,虽然旋转了之后可能会有部分保留这个比例,也能找到,但如果是拍照的,除了宣传还有一些透视的话,那么opencv很容易就检测不到,就像这张图
![角点](https://blog.357573.com/2020/07/03/opencv实现二维码检测/1.jpg)  
因为它有透视的弯折,导致了这个比例不一定正确,或者图像不清晰的时候,严格的去数点很容易就找不到了,所以opencv的这个方法检出率极低.
所以我自己开发的时候没有使用他的这个方法.

# 自己动手实现
我用的是找轮廓的方法,根据轮廓层级,找到3层的轮廓,然后作为候选,就不去数点确认比例了.

### getContours
看我的步骤
```
    path = 'data/qrcode.jpg'
    img = cv2.imread(path)
    thresholdImage, contours, hierarchy = getContours(img)
```
首先把轮廓提取出来
```
def convert_img_to_binary(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    binary_img = cv2.adaptiveThreshold(
            gray,
            255,                    # Value to assign
            cv2.ADAPTIVE_THRESH_MEAN_C,# Mean threshold
            cv2.THRESH_BINARY,
            11,                     # Block size of small area
            2,                      # Const to substract
        )
    return binary_img

def getContours(img):
    binary_img = convert_img_to_binary(img)
    thresholdImage = cv2.Canny(binary_img, 100, 200) #Edges by canny edge detection
    _, contours, hierarchy = cv2.findContours(
            thresholdImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return thresholdImage, contours, hierarchy
```
把图片预处理了一下,首先二值化,然后做一下canny提取边缘信息,就成了这样
![canny](https://blog.357573.com/2020/07/03/opencv实现二维码检测/canny.jpg)  
,然后调opencv的<code>findContours</code>函数,找到轮廓,
![轮廓](https://blog.357573.com/2020/07/03/opencv实现二维码检测/contours.jpg)  
结果发现全是轮廓,哈哈,不急,我们慢慢挑,最重要的是最后那个返回值,也就是层级关系.这个是我们算法的核心信息.

然后我们就要开始利用这个层级关系了,因为二维码轮廓是黑白黑三层的,所以我们搜索所有三层的轮廓
```
    # qrcode corner has 3 levels
    levelsNum = 3
    patterns, patternsIndices = getContourWithinLevel(levelsNum, contours, hierarchy)
```
看一下这个怎么搜索的
### getContourWithinLevel
```
def isPossibleCorner(contourIndex, levelsNum, contours, hierarchy):
    # if no chirld, return -1
    chirldIdx = hierarchy[0][contourIndex][2] 
    level = 0
    while chirldIdx != -1:
        level += 1
        chirldIdx = hierarchy[0][chirldIdx][2] 
    if level >= levelsNum:
        return checkRatioOfContours(contourIndex, contours, hierarchy)
    return False

def getContourWithinLevel(levelsNum, contours, hierarchy):
    # find contours has 3 levels
    patterns = []
    patternsIndices = []
    for contourIndex in range(len(contours)):
        if isPossibleCorner(contourIndex, levelsNum, contours, hierarchy):
            patterns.append(contours[contourIndex])
            patternsIndices.append(contourIndex)
    return patterns, patternsIndices
```
也就是找到有3层子轮廓的轮廓,把它返回回来,其中<code>hierarchy</code>这个对象,固定的shape是<code>1,n,4</code>其中n是轮廓的数量,例如我们这个例子就是<code>(1, 1253, 4)</code>,最后那个4个维度代表<code>下一个轮廓id,上一个轮廓id,子轮廓id,父轮廓id</code>如果没有的话统统都是-1,所以我们这里获得<code>hierarchy[0][contourIndex][2]</code>就是获取了子轮廓的id,然后就数连着3级都有子轮廓就存下来,但是也不是都存下来了,因为最后还有一个<code>checkRatioOfContours</code>函数,这个是防止子轮廓大小比例异常
```
def checkRatioOfContours(index, contours, hierarchy):
    firstChildIndex = hierarchy[0][index][2]
    secondChildIndex = hierarchy[0][firstChildIndex][2]
    firstArea = cv2.contourArea(contours[index]) / (
        cv2.contourArea(contours[firstChildIndex]) + 1e-5)
    secondArea = cv2.contourArea(contours[firstChildIndex]) / (
        cv2.contourArea(contours[secondChildIndex]) + 1e-5)
    return ((firstArea / (secondArea+ 1e-5)) > 1 and \
            ((firstArea / (secondArea+ 1e-5)) < 10))
```
所以我比较了一下,父轮廓的大小要在子轮廓的1-10倍之间,不然就算噪声排除掉了.
现在看看还剩下多少轮廓
![轮廓](https://blog.357573.com/2020/07/03/opencv实现二维码检测/contours2.jpg)  
这样一挑,就只剩下8个轮廓了.
如果图片特别模糊,看不清是3个层级怎么办,我们后续也加了一步
```
    #in case not all the picture has clear pattern
    while len(patterns) < 3 and levelsNum > 0:
        levelsNum -= 1
        patterns, patternsIndices = getContourWithinLevel(levelsNum, contours, hierarchy)
```
如果找到的关键点还不够3个,那么我们就减小层级,这样假设太小了的,三层只能看见两层的我们也能找到,所以我们逐步缩小了层级,直到找够3个为止
找到之后就开始处理了,如果此时还不3个就直接宣告GG吧,也不浪费时间了
```
    interstingPatternList = []
    if len(patterns) < 3 :
        print('no enough pattern')
        return False, []
        # return False
```
如果刚好找到了3个,那就把这3个都作为感兴趣的轮廓加进去
```
    elif len(patterns) == 3:
        for patternIndex in range(len(patterns)):
            x, y, w, h = cv2.boundingRect(patterns[patternIndex])
            interstingPatternList.append(patterns[patternIndex])

        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        show(img, 'qrcode')
        # return patterns
```
如果比3个多,我们就要把父轮廓提取出来,刚才那8个都是一个父轮廓带着一个子轮廓的,现在我们要把这个父轮廓提取出来,子轮廓就不要了.
我们来挨个判断一下,把那些没有爸爸的加到感兴趣的里面,但是这样有个问题,如果有的有爸爸,但是他的爸爸在早期就被淘汰了,例如有的图整张图片有个超大的圈,所有图案都是外面那个圈的子轮廓,这时候我们就不能从全局去找爸爸了,那该怎么办呢?我们就从这8个待选的轮廓中找爸爸.
```
    elif len(patterns) > 3:
        # sort from small to large
        patternAreaList = np.array(
                [cv2.contourArea(pattern) for pattern in patterns])
        areaIdList = np.argsort(patternAreaList)
        # get patterns without parents
        intrestingPatternIdList = []
        for i in range(len(areaIdList) - 1, 0, -1):
            index = patternsIndices[areaIdList[i]]
            if hierarchy[0][index][3] == -1:
                intrestingPatternIdList.append(index)
            else:
                # We can make sure the parent must appear before chirld because we sorted the list by area
                if not isParentInList(intrestingPatternIdList, index, hierarchy):
                    intrestingPatternIdList.append(index)

        for intrestingPatternId in intrestingPatternIdList:
            x, y, w, h = cv2.boundingRect(contours[intrestingPatternId])
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            interstingPatternList.append(contours[intrestingPatternId])
        show(img, 'qrcode')
```
我们先给这些轮廓按照大小排个序,这样的话从大到小来判断,就可以优先提取出最大的了,剩下的绝对不可能是前一个的父轮廓,所以我们每个轮廓就判断一下它有没有爸爸已经被我们选中了即可.
这下我们搜集的所有轮廓都不互为父子了.
![轮廓](https://blog.357573.com/2020/07/03/opencv实现二维码检测/contours3.jpg) 
我们再看现在剩下的轮廓,有一个在小电视上,那是我们不希望存在的,现在就要想办法把它除掉.
首先使用距离肯定是不合适的,因为小电视明显和最下面的那个最近,用全部距离的话会把右上角那个点去掉,把小电视保留.
那么我们就用角度好了,每三个点都找一遍,选出两条垂直并且长度也差不多的线
现在是轮廓,我们要把它变成点
首先每个轮廓找到他自己的重心
```
    centerOfMassList = getCenterOfMass(interstingPatternList)
    for centerOfMass in centerOfMassList:
        cv2.circle(img_show, tuple(centerOfMass), 3, (0, 255, 0))
```
结果如下
![重心](https://blog.357573.com/2020/07/03/opencv实现二维码检测/mass.jpg)
找重心的函数是
```
def getCenterOfMass(contours):
    pointList = []
    for i in range(len(contours)):
        moment = cv2.moments(contours[i])
        centreOfMassX = int(moment['m10'] / moment['m00'])
        centreOfMassY = int(moment['m01'] / moment['m00'])
        pointList.append([centreOfMassX, centreOfMassY])
    return pointList
```

我是通过计算图像的矩来找的重心,说白了也就是哪边点多就往哪边偏移,这符合重心的原理.当然直接用轮廓找个重心也是可以的,但是这样噪声影响比较大.
找到重心之后我们要挑出最终的三个点
```
    id1, id2, id3 = 0, 1, 2
    if len(patterns) > 3:
        result = selectPatterns(centerOfMassList)
        if result is None:
            print('no correct pattern')
            return False, []
        id1, id2, id3 = result
```
### selectPatterns
```
def selectPatterns(pointList):
    lineList = []
    for i in range(len(pointList)):
        for j in range(i, len(pointList)):
            lineList.append([i, j])
    finalLineList = []
    finalResult = None
    minLengthDiff = -1
    for i in range(len(lineList)):
        for j in range(i, len(lineList)):
            line1 = np.array([pointList[lineList[i][0]][0] -  pointList[lineList[i][1]][0], 
                pointList[lineList[i][0]][1] -  pointList[lineList[i][1]][1]])
            line2 = np.array([pointList[lineList[j][0]][0] -  pointList[lineList[j][1]][0], 
                pointList[lineList[j][0]][1] -  pointList[lineList[j][1]][1]])
            pointIdxList = np.array([lineList[i][0], lineList[i][1], lineList[j][0], lineList[j][1]])
            pointIdxList = np.unique(pointIdxList)
            # print('****')
            if len(pointIdxList) == 3:
                theta = lineAngle(line1, line2)
                if abs(math.pi / 2 - theta) < math.pi / 6:
                    lengthDiff = abs(np.linalg.norm(line1, axis = 0) - np.linalg.norm(line2, axis = 0))
                    if  lengthDiff < minLengthDiff or minLengthDiff < 0:
                        minLengthDiff = abs(np.linalg.norm(line1, axis = 0) - np.linalg.norm(line2, axis = 0))
                        finalResult = pointIdxList

    
    return finalResult
```
我的做法是每3个点做两条线段,然后判断线段如果差不多90度,我这里的范围比较宽松,可以浮动30度,也就是说你夹角只要在60-120度之间的都可以进入下一步,下一步就是从所有夹角符合的情况中选出线段长度最接近的,也就是<code>minLengthDiff</code>这一组,恭喜这三个点两条线获胜,就是我们想要的轮廓.

最后就是来计算旋转角度了,也就是看这三个点哪个是左下,哪个是左上,哪个是右上
```
    interstingPatternList = np.array(interstingPatternList)[[id1, id2, id3]]
    centerOfMassList = np.array(centerOfMassList)[[id1, id2, id3]]
    pointList = getOrientation(interstingPatternList, centerOfMassList)
```
### getOrientation
```
def getOrientation(contours, centerOfMassList):
        distance_AB = np.linalg.norm(centerOfMassList[0].flatten() - centerOfMassList[1].flatten(), axis = 0)
        distance_BC = np.linalg.norm(centerOfMassList[1].flatten() - centerOfMassList[2].flatten(), axis = 0)
        distance_AC = np.linalg.norm(centerOfMassList[0].flatten() - centerOfMassList[2].flatten(), axis = 0)

        largestLine = np.argmax(
            np.array([distance_AB, distance_BC, distance_AC]))
        bottomLeftIdx = 0
        topLeftIdx = 1
        topRightIdx = 2
        if largestLine == 0:
            bottomLeftIdx, topLeftIdx, topRightIdx = 0, 2, 1 
        if largestLine == 1:
            bottomLeftIdx, topLeftIdx, topRightIdx = 1, 0, 2 
        if largestLine == 2:
            bottomLeftIdx, topLeftIdx, topRightIdx = 0, 1, 2 

        # distance between point to line:
        # abs(Ax0 + By0 + C)/sqrt(A^2+B^2)
        slope = (centerOfMassList[bottomLeftIdx][1] - centerOfMassList[topRightIdx][1]) / (centerOfMassList[bottomLeftIdx][0] - centerOfMassList[topRightIdx][0] + 1e-5)
        # y = kx + b => AX + BY +C = 0 => B = 1, A = -k, C = -b
        coefficientA = -slope
        coefficientB = 1
        constant = slope * centerOfMassList[bottomLeftIdx][0] - centerOfMassList[bottomLeftIdx][1]
        distance = (coefficientA * centerOfMassList[topLeftIdx][0] + coefficientB * centerOfMassList[topLeftIdx][1] + constant) / (
            np.sqrt(coefficientA ** 2 + coefficientB ** 2))


        pointList = np.zeros(shape=(3,2))
        # 回    回   tl   bl
        if (slope >= 0) and (distance >= 0):
            # if slope and distance are positive A is bottom while B is right
            if (centerOfMassList[bottomLeftIdx][0] > centerOfMassList[topRightIdx][0]):
                pointList[1] = centerOfMassList[bottomLeftIdx]
                pointList[2] = centerOfMassList[topRightIdx]
            else:
                pointList[1] = centerOfMassList[topRightIdx]
                pointList[2] = centerOfMassList[bottomLeftIdx]
            # TopContour in the SouthWest of the picture
            ORIENTATION = "SouthWest"

        # 回   回     bl     tl
        #
        #      回            tr
        elif (slope > 0) and (distance < 0):
            # if slope is positive and distance is negative then B is bottom
            # while A is right
            if (centerOfMassList[bottomLeftIdx][1] > centerOfMassList[topRightIdx][1]):
                pointList[2] = centerOfMassList[bottomLeftIdx]
                pointList[1] = centerOfMassList[topRightIdx]
            else:
                pointList[2] = centerOfMassList[topRightIdx]
                pointList[1] = centerOfMassList[bottomLeftIdx]
            ORIENTATION = "NorthEast"


        #       回            bl
        #
        # 回    回      tr    tl
        elif (slope < 0) and (distance > 0):
            if (centerOfMassList[bottomLeftIdx][0] > centerOfMassList[topRightIdx][0]):
                pointList[1] = centerOfMassList[bottomLeftIdx]
                pointList[2] = centerOfMassList[topRightIdx]
            else:
                pointList[1] = centerOfMassList[topRightIdx]
                pointList[2] = centerOfMassList[bottomLeftIdx]
            ORIENTATION = "SouthEast"
        # 回    回    tl   tr
        #
        # 回          bl
        elif (slope < 0) and (distance < 0):

            if (centerOfMassList[bottomLeftIdx][0] > centerOfMassList[topRightIdx][0]):
                pointList[2] = centerOfMassList[bottomLeftIdx]
                pointList[1] = centerOfMassList[topRightIdx]
            else:
                pointList[2] = centerOfMassList[topRightIdx]
                pointList[1] = centerOfMassList[bottomLeftIdx]
        pointList[0] = centerOfMassList[topLeftIdx]
        return pointList
```
我这里其实就是先用距离找出离两个距离最远的点,剩下一个就是直角点了,然后算出斜边的斜率,再算出直角点在斜边的哪一侧,就可以得出这个图形的旋转方式.
最终就可以得到我们想要的三个点了,我返回时候是按照左上角,左下角,右上角的这个顺序返回的.
看看识别结果
![结果](https://blog.357573.com/2020/07/03/opencv实现二维码检测/result.jpg)
![结果](https://blog.357573.com/2020/07/03/opencv实现二维码检测/result2.jpg)
项目我是开源了,放在了github上
[https://github.com/keefeWu/QRCode](https://github.com/keefeWu/QRCode)