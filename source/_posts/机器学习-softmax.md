---
title: 机器学习-softmax
date: 2020-02-04 16:22:44
tags: [机器学习]
categories: 
- 机器学习
---
说到softmax呢，首先得提到伯努利分布(Bernoulli distribution)伯努利分布很简单，也叫二项分布，就是只有两种结果的分布。比如说扔硬币，正面的概率是0.5，那么P(正) = 0.5P(反) = 0.5再举个例子，如果打靶射中的概率是0.8，则P(中) = 0.8P(不中) = 0.2这种二分类的问题就是伯努利分布，如果是多种分类呢，那么叫多伯努利分布了(multinoulli distribution)，也叫范畴分布(categorical distribution)比方说有k种状态，小明今天的午餐有三种选择，披萨，米饭和汉堡。他对每种的兴趣比例依次为3:5:2那么午餐吃每一样的概率分别是0.3,0.5和0.2这种所有概率和为1的多重分布就是softmax了。有多少种类型就有多少维的长度，每一维代表这一个类别的概率，一般分类算法就会选择概率最高的一个分类。