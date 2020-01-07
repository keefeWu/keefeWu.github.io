---
title: python机器学习-numpy常用函数
date: 2020-01-07 19:07:28
tags: [python,numpy]
categories: 
- numpy
---
#### 根据id操作元素

```
a=np.array([3,4,5,6,7,8,9])
```

> array([3, 4, 5, 6, 7, 8, 9])
```
array([3, 4, 5, 6, 7, 8, 9])
```
> [2, 3, 4]

```
a[id]
```

> array([5, 6, 7])
#### 深度拷贝

```
import copy
b = copy.deepcopy(a)
```

> array([3, 4, 5, 6, 7, 8, 9])
```
a[id] = 0
```
> array([3, 4, 0, 0, 0, 8, 9])

#### 反选操作

```
c = b - a
```

> array([0, 0, 5, 6, 7, 0, 0])
### 向上取整

```
np.ceil(a)
```
### 向下取整

```
np.floor(a)
```

### 修改类型

```
b = a.astype('int64')
```
### 计数排序数组

```
counts = np.bincount(cloud_line)
```
### 求众数

```
max_idx = np.argmax(counts)
```
### 求众数出现的次数或数组最大值

```
np.max(counts)
```
### 计算两向量的欧氏距离

```
np.linalg.norm(a - b, axis = 1)
```

