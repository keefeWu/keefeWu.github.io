---
title: caffe源码阅读《二》softmax层
date: 2020-02-08 15:10:49
tags:
---
前传代码

    
    
    template <typename Dtype>
    void SoftmaxLayer<Dtype>::Forward_cpu(const vector<Blob<Dtype>*>& bottom,
        const vector<Blob<Dtype>*>& top) {
      const Dtype* bottom_data = bottom[0]->cpu_data();
      Dtype* top_data = top[0]->mutable_cpu_data();
      Dtype* scale_data = scale_.mutable_cpu_data();
      int channels = bottom[0]->shape(softmax_axis_);
      int dim = bottom[0]->count() / outer_num_;
      caffe_copy(bottom[0]->count(), bottom_data, top_data);
      // We need to subtract the max to avoid numerical issues, compute the exp,
      // and then normalize.
      /*************这里遍历找出上一层的最大值*****************/
      for (int i = 0; i < outer_num_; ++i) {
        // initialize scale_data to the first plane
        caffe_copy(inner_num_, bottom_data + i * dim, scale_data);
        for (int j = 0; j < channels; j++) {
          for (int k = 0; k < inner_num_; k++) {
            scale_data[k] = std::max(scale_data[k],
                bottom_data[i * dim + j * inner_num_ + k]);
          }
        }
        /***找到最大值放在scale_data这个数组里，一般一张图片只有一个分类，这个数组其实只有一个数***/
        // subtraction
        caffe_cpu_gemm<Dtype>(CblasNoTrans, CblasNoTrans, channels, inner_num_,
            1, -1., sum_multiplier_.cpu_data(), scale_data, 1., top_data);
        // exponentiation
        caffe_exp<Dtype>(dim, top_data, top_data);
        // sum after exp
        caffe_cpu_gemv<Dtype>(CblasTrans, channels, inner_num_, 1.,
            top_data, sum_multiplier_.cpu_data(), 0., scale_data);
        // division
        for (int j = 0; j < channels; j++) {
          caffe_div(inner_num_, top_data, scale_data, top_data);
          top_data += inner_num_;
        }
      }
    }
    

由于softmax的前向算法是bottom层所有的元素作为以e为底的指数进行求和，例如有100个分类，那么上面一个全连接层的输出为x1,x2,...x100。

那么softmax输出的是每个分类的概率，求法就是![y_{i}=\\frac{e^{xi}}{\\sum_{j}^{n}e^{xj}}](0.latex)，也就是要先求出每个输入的e为底的值，然后求和，每个的概率就是刚求的值/和。这样保证了所有概率加起来肯定是1.

这里由于要求指数函数，值可能会很大，所以caffe先找出最大的输入，然后每个值减去它之后再求指数，因为e为底的指数函数是单调递增的，并且输入为0的时候就是1了，所以我们减去最大的，也就能保证算出来的都是小于1的数，也就不用担心溢出的问题了。

caffe_cpu_gemm 这个函数是调用的cblas里算矩阵乘法的函数，caffe使用这个来做减法，也就是减去最大值。

然后caffe_exp求幂，也就是exp(xi)这一步

caffe_cpu_gemv来求和

然后caffe_div求每一项的概率，存入top_data中。

其中inner_num是指的每一张图片对应的分类数，outer_num是batch_size，scale层原本是存储过渡数据，也就是从上到下传递不改变的数据的，这里scale_data则用来作为临时变量存储中间结果。

**********************************************************分割线*****************************************************************************

反向代码

计算loss

    
    
    template <typename Dtype>
    void SoftmaxWithLossLayer<Dtype>::Forward_cpu(
        const vector<Blob<Dtype>*>& bottom, const vector<Blob<Dtype>*>& top) {
      // The forward pass computes the softmax prob values.
      softmax_layer_->Forward(softmax_bottom_vec_, softmax_top_vec_);
      const Dtype* prob_data = prob_.cpu_data();
      const Dtype* label = bottom[1]->cpu_data();
      int dim = prob_.count() / outer_num_;
      int count = 0;
      Dtype loss = 0;
      for (int i = 0; i < outer_num_; ++i) {
        for (int j = 0; j < inner_num_; j++) {
          const int label_value = static_cast<int>(label[i * inner_num_ + j]);
          if (has_ignore_label_ && label_value == ignore_label_) {
            continue;
          }
          DCHECK_GE(label_value, 0);
          DCHECK_LT(label_value, prob_.shape(softmax_axis_));
          loss -= log(std::max(prob_data[i * dim + label_value * inner_num_ + j],
                               Dtype(FLT_MIN)));
          ++count;
        }
      }
      top[0]->mutable_cpu_data()[0] = loss / get_normalizer(normalization_, count);
      if (top.size() == 2) {
        top[1]->ShareData(prob_);
      }
    }
    

首先softmax的loss计算方法是

![L=-\\sum y_{i} * logs_{i}](1.latex)

其中si就是之前求得那个概率，yi其实只有两个值，要么是0要么是1，也就是对应类别的概率。

那么这个公式实际上就可以简化成L = -logsi

也就是说正确的那个你预测的概率越大，也就是预测的越准确，损失越小。

这点在代码里很容易找到，其中有一点，caffe为了防止预测概率为0，设置了一个概率最小值，也就是FLT_MIN

1.17549435E-38F

这样算出的loss是87.3365，所以我们训练的时候经常会碰到这个。

这个原因其实是概率为0了，之所以为0不是因为真的是0，而是之前的特征太大了，exp()这个特征就直接爆炸了，导致float成了一个很小的数。所以输入数据一定要归一化，学习率也不宜过大，不然也容易乱跳跳到一个很大的权值。

