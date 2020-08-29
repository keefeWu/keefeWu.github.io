---
title: pytorch详解<一>数据流
date: 2020-08-20 09:48:20
tags:
---
# 序言
pytorch训练时候的数据是提供了读取数据类的,一共有三个
* Dataset
* DataLoader
* DataLoaderIter

这三个类是pytorch实现好了的,我们可以直接拿来用,也可以当做基类,继承之后自己实现.

# Dataset
这个类是最基础的,我们需要实现最基础的两个功能
* __getitem__ 读数据和标签
* __len__ 数据集的长度

## __getitem__
```
@param1: index 当前读取的这个数据的序号
```
```
path = self.pathList[index]
img = Image.open(path).convert("RGB")
size = min(img.height, img.width)
img = img.crop((0,0,size,size))
if self.transform:
    img = self.transform(img)
return img
```
我这里的实现是用的PIL读取图片,然后把图片裁剪成了正方形,执行pytorch自带的transform函数,这个transform也是我传进来的,具体的声明在这里
```
normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])

transform = transforms.Compose([
                                            transforms.Resize(224),
                                            transforms.ToTensor(),
                                            normalize,
                                            ])
```
对于有些算法,希望图片随机裁剪依然能够识别的,可以使用随机裁剪`RandomResizedCrop`,还有随机对称`RandomHorizontalFlip`
