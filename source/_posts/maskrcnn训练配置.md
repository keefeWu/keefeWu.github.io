---
title: maskrcnn训练配置
date: 2020-02-19 21:58:30
tags:
---

# 训练
```
rm log.txt
cp maskrcnn_benchmark/config/paths_catalog.py /maskrcnn-benchmark/maskrcnn_benchmark/config/
cp configs/e2e_mask_rcnn_R_50_FPN_1x.yaml /maskrcnn-benchmark/configs/
python tools/train_net.py --config-file "configs/e2e_mask_rcnn_R_50_FPN_1x.yaml" SOLVER.IMS_PER_BATCH 2 SOLVER.BASE_LR 0.0025 SOLVER.MAX_ITER 720000 SOLVER.STEPS "(480000, 640000)" TEST.IMS_PER_BATCH 1 MODEL.RPN.FPN_POST_NMS_TOP_N_TRAIN 2000
```

# 配置
configs/e2e_mask_rcnn_R_50_FPN_1x.yaml
## 默认值
maskrcnn_benchmark/config/defaults.py

# 设置数据路径
maskrcnn_benchmark/config/paths_catalog.py
# 裁剪模型
## 默认下载地址
default="~/.torch/models/"
## 裁剪方式
tools/trim.py
## 如果裁剪之后还说不匹配
看看是不是之前用了别的模型保存了，他会默认继续训练，所以如果之前别的模型输出数量不一致就会不匹配，删掉或者换个目录就行了