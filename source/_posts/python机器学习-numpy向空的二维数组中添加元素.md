---
title: python机器学习-numpy向空的二维数组中添加元素
date: 2020-02-05 10:44:34
tags: [运维]
categories: 
- 运维
---
直接上代码了
```
x = np.empty(shape=[0, 4], int)
x = np.append(x, [[1,2,3,4]], axis = 0)
x = np.append(x, [[1,2,3,4]], axis = 0)
```
这样就添加了两行4列的数据了。注意append里面是两层括号，这个非常重要，如果漏掉了就不是二维数组了，用axis的时候就会报维度不匹配。