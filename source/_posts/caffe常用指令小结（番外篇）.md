---
title: caffe常用指令小结（番外篇）
date: 2020-02-08 15:21:53
tags:
---
<http://blog.csdn.net/u011021773/article/details/53782969>  

训练：/caffe/build/tools/caffe train --solver=solver.prototxt

以下指令皆是附加参数，跟在训练后面即可

【

输出到日志：2>&1 | tee $LOG（其中LOG文件名为时间则 LOG=./log/train-`date
+%Y-%m-%d-%H-%M-%S`.log

读取保存文件继续训练： -snapshot=./snape/snape_iter_200000.solverstate

】

  

数据预处理部分：

【

图片转换成IMDB：$caffe_path/build/tools/convert_imageset $data_root_path $nameFile
$project_path/$dbname 0

计算均值： $caffe_path/build/tools/compute_image_mean $dbname $meanfilename

绘制网络结构：$caffe_path/build/tools/compute_image_mean $solverFile $picName

】

