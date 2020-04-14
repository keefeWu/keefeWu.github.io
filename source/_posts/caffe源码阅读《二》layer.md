---
title: caffe源码阅读《二》layer
date: 2020-02-25 09:41:56
top: 2
tags: [caffe]
categories: 
- caffe源码解读
---

首先layer这个类是一个基类，所以他是没有cpp实现的，可以看一下它的cpp代码
```
#include "caffe/layer.hpp"

namespace caffe {

INSTANTIATE_CLASS(Layer);

}  // namespace caffe
```
除了注册函数之外一无所有，所以他的全部宝藏都在hpp文件里，我们直接在include目录查看他的hpp文件。

打开hpp文件，首先看到的是一个很有意思的注释
```
/**
 Forward declare boost::thread instead of including boost/thread.hpp
 to avoid a boost/NVCC issues (#1009, #1010) on OSX.
 */
 ```
 它说用<code>boost::thread</code>这个声明来代替引用<code>boost/thread.hpp</code>头文件，以解决<code>boost/NVCC</code>的issues(#1009, #1010)，有兴趣的可以去github上看看这两个issue。
它紧接着是
```
namespace boost { class mutex; }
```
声明了这个类，这个类就是和那个问题有关，如果遇到了有兴趣的可以去研究一下。
好进入正题，这里有个简短的介绍。
```
/**
 * @brief An interface for the units of computation which can be composed into a
 *        Net.
 *
 * Layer%s must implement a Forward function, in which they take their input
 * (bottom) Blob%s (if any) and compute their output Blob%s (if any).
 * They may also implement a Backward function, in which they compute the error
 * gradients with respect to their input Blob%s, given the error gradients with
 * their output Blob%s.
 */
```
* 介绍了一下这个layer类，它是把各种简短的单元连接成一个网络的接口。layer这个类的子类必须实现<code>Forward</code>函数。
* 用bottom Blob作为输入，计算输出的blob。
* 它们也许会实现一个<code>Backward</code>函数，这里用的是也许了，也许的意思就是说也可以不用实现反传函数。因为不是每个层都会有反传的，最典型的就是输入层，它已经是最上层了，所以不需要反传给任何层。
* 用输入的blob去计算梯度误差，然后把这个梯度误差传给输出的blob

接着往下看，就要进入这个类的实现了
```
template <typename Dtype>
```
这里定义了一个模板类型，叫<code>Dtype</code>,这个Dtype呢其实就是个<code>float</code>它是在之前的函数注册的时候指定的，可以选择<code>float</code>或者<code>double</code>，这个是caffe的工厂模式的性质，之后统一介绍。
# 构造函数Layer
```
  explicit Layer(const LayerParameter& param)
```
这里声明了一个显式类型的构造函数，C++里面有一种用法，就是声明一个layer的对象直接等于一个<code>LayerParameter</code>的参数，它就可以直接构造这个对象了。这个就叫做隐式转换，如果没有声明<code>explicit</code>的构造函数全部默认为隐式转换，这样的话假设有多个构造函数，但是参数类型模棱两可时候就很容易出现歧义了，比方说<code>int</code>和<code>char</code>类型，本身就有很大的交集，这样你直接一个类=一个字母，它不知道是根据ascii码调用int类型的函数还是根据字符调用字符类型的函数。所以加上一个<code>explicit</code>，强制执行显示声明，必须老老实实的<code>类名 对象名(参数)。

他的实现
## 拷贝phase
```
phase_ = param.phase();
```
## 拷贝blob
```
      if (layer_param_.blobs_size() > 0) {
        blobs_.resize(layer_param_.blobs_size());
        for (int i = 0; i < layer_param_.blobs_size(); ++i) {
          blobs_[i].reset(new Blob<Dtype>());
          blobs_[i]->FromProto(layer_param_.blobs(i));
        }
      }
```
把上面一层的blob和下面一层的blob都拷贝进来。
# Setup函数
注释里说了
```
  /**
   * @brief Implements common layer setup functionality.
   *
   * @param bottom the preshaped input blobs
   * @param top
   *     the allocated but unshaped output blobs, to be shaped by Reshape
   *
   * Checks that the number of bottom and top blobs is correct.
   * Calls LayerSetUp to do special layer setup for individual layer types,
   * followed by Reshape to set up sizes of top blobs and internal buffers.
   * Sets up the loss weight multiplier blobs for any non-zero loss weights.
   * This method may not be overridden.
   */
```
简单的实现了公有的层的setup函数，一共有两个参数
* bottom层的已经预分配形状的输入blob
* top层，还没有分配形状的blob，需要在这里对它进行塑性
也就是说他的输入是固定的，但是输出是需要根据输入以及这一层的参数决定的，比方说卷积层就要根据步长来决定输出缩小了几倍。
它的具体步骤在这个注释里也写到了。
* 检查bottom层和top层的数量是否正确。
* 调用<code>LayerSetup</code>去对每一层做特定化的setup，针对不同层使用不同的方法。根据上一层的形状去设置下一层的blob的大小，还有缓存空间等等都给分配好。
* 设置loss的权重和做乘法用的单元的blob
* 这个函数就是个抽象的流程，你最好不要重写这个Setup函数了。
看看他的实现
```
  void SetUp(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top) {
    CheckBlobCounts(bottom, top);
    LayerSetUp(bottom, top);
    Reshape(bottom, top);
    SetLossWeights(top);
  }
```
这个函数就是个大纲，我们严格遵循最后的原则，想要实现就实现函数内容的具体的步骤，它的这个大纲就不要动了。
# LayerSetUp 函数
首先还是看注释
```
  /**
   * @brief Does layer-specific setup: your layer should implement this function
   *        as well as Reshape.
   *
   * @param bottom
   *     the preshaped input blobs, whose data fields store the input data for
   *     this layer
   * @param top
   *     the allocated but unshaped output blobs
   *
   * This method should do one-time layer specific setup. This includes reading
   * and processing relevent parameters from the <code>layer_param_</code>.
   * Setting up the shapes of top blobs and internal buffers should be done in
   * <code>Reshape</code>, which will be called before the forward pass to
   * adjust the top blob sizes.
   */
```
他首先说自己是个明确的setup函数，然后又说你的层应该实现这个函数，那意思就是说这里还是只是一个声明，作为一个虚函数，具体的需要到具体的类里面去实现它。
然后看他的两个参数，和<code>Setup</code>是一样的参数，都是传入一个blob，然后去分配传出的blob。
# Reshape 函数
这个<code>Reshape</code>也是一个抽象函数，看他的注释
```
  /**
   * @brief Adjust the shapes of top blobs and internal buffers to accommodate
   *        the shapes of the bottom blobs.
   *
   * @param bottom the input blobs, with the requested input shapes
   * @param top the top blobs, which should be reshaped as needed
   *
   * This method should reshape top blobs as needed according to the shapes
   * of the bottom (input) blobs, as well as reshaping any internal buffers
   * and making any other necessary adjustments so that the layer can
   * accommodate the bottom blobs.
  */
```
连参数都是一样的，他的作用是调整top blob的形状和分配bottom blob的缓存区大小。

# Forward 函数
前传函数，这个是重头戏了
```
  /**
   * @brief Given the bottom blobs, compute the top blobs and the loss.
   *
   * @param bottom
   *     the input blobs, whose data fields store the input data for this layer
   * @param top
   *     the preshaped output blobs, whose data fields will store this layers'
   *     outputs
   * \return The total loss from the layer.
   *
   * The Forward wrapper calls the relevant device wrapper function
   * (Forward_cpu or Forward_gpu) to compute the top blob values given the
   * bottom blobs.  If the layer has any non-zero loss_weights, the wrapper
   * then computes and returns the loss.
   *
   * Your layer should implement Forward_cpu and (optionally) Forward_gpu.
   */
```
他的作用是根据bottom层计算出top层的blob和loss
参数也是同样的这两个，返回值是这一层总的loss。
Forward函数是根据设备调用不同的函数，也就是cpu就调用cpu的函数，gpu就调用gpu的函数，通过输入计算输出的值，如果这个层有任何非零的loss权重那么这个层就要算一个loss出来返回。你的层必须实现<code>Forward_cpu</code>,如果有必要的话也可以实现<code>Forward_gpu</code>。
所以他这个函数也是一个虚函数，就是等着你去实现它。

# Backward 函数
接着是反传函数，这个反传函数也是个虚函数，我们还是先来看他的注释
```
  /**
   * @brief Given the top blob error gradients, compute the bottom blob error
   *        gradients.
   *
   * @param top
   *     the output blobs, whose diff fields store the gradient of the error
   *     with respect to themselves
   * @param propagate_down
   *     a vector with equal length to bottom, with each index indicating
   *     whether to propagate the error gradients down to the bottom blob at
   *     the corresponding index
   * @param bottom
   *     the input blobs, whose diff fields will store the gradient of the error
   *     with respect to themselves after Backward is run
   *
   * The Backward wrapper calls the relevant device wrapper function
   * (Backward_cpu or Backward_gpu) to compute the bottom blob diffs given the
   * top blob diffs.
   *
   * Your layer should implement Backward_cpu and (optionally) Backward_gpu.
   */
```
简介：给一个top blob的梯度误差来计算bottom blob的梯度误差
## 参数
* top层的blob
* propagate_down bool类型的变量，用来决定要不要反传的状态
* bottom层的blob

# 返回成员变量的函数
有很多返回成员变量的函数，他们的名字就是变量的名字，例如<code>blobs</code><code>layer_param</code>
```
  /**
   * @brief Returns the vector of learnable parameter blobs.
   */
  vector<shared_ptr<Blob<Dtype> > >& blobs() {
    return blobs_;
  }

  /**
   * @brief Returns the layer parameter.
   */
  const LayerParameter& layer_param() const { return layer_param_; }
```

# ToProto 函数
他也是一个虚函数
```
  /**
   * @brief Writes the layer parameter to a protocol buffer
   */
  virtual void ToProto(LayerParameter* param, bool write_diff = false);
```
它的作用是把缓存里的参数写道proto文件里

# loss 函数
他的作用就是返回对应blob的loss了
```
  /**
   * @brief Returns the scalar loss associated with a top blob at a given index.
   */
  inline Dtype loss(const int top_index) const {
    return (loss_.size() > top_index) ? loss_[top_index] : Dtype(0);
  }
```
可以看到他做了一个范围的判断，如果输入还没有loss多，那么就把多出来的loss设置成0

# set_loss 函数
顾名思义啊，这个函数就是用来设置loss的初始值的。
```
  /**
   * @brief Sets the loss associated with a top blob at a given index.
   */
  inline void set_loss(const int top_index, const Dtype value) {
    if (loss_.size() <= top_index) {
      loss_.resize(top_index + 1, Dtype(0));
    }
    loss_[top_index] = value;
  }
```
首先resize了<code>loss_</code>这个vector的size，保证和输入的是一致的，然后就开始赋值了。

# 返回各种状态的函数
```
  /**
   * @brief Returns the layer type.
   */
  virtual inline const char* type() const { return ""; }

  /**
   * @brief Returns the exact number of bottom blobs required by the layer,
   *        or -1 if no exact number is required.
   *
   * This method should be overridden to return a non-negative value if your
   * layer expects some exact number of bottom blobs.
   */
  virtual inline int ExactNumBottomBlobs() const { return -1; }
  /**
   * @brief Returns the minimum number of bottom blobs required by the layer,
   *        or -1 if no minimum number is required.
   *
   * This method should be overridden to return a non-negative value if your
   * layer expects some minimum number of bottom blobs.
   */
  virtual inline int MinBottomBlobs() const { return -1; }
  /**
   * @brief Returns the maximum number of bottom blobs required by the layer,
   *        or -1 if no maximum number is required.
   *
   * This method should be overridden to return a non-negative value if your
   * layer expects some maximum number of bottom blobs.
   */
  virtual inline int MaxBottomBlobs() const { return -1; }
  /**
   * @brief Returns the exact number of top blobs required by the layer,
   *        or -1 if no exact number is required.
   *
   * This method should be overridden to return a non-negative value if your
   * layer expects some exact number of top blobs.
   */
  virtual inline int ExactNumTopBlobs() const { return -1; }
  /**
   * @brief Returns the minimum number of top blobs required by the layer,
   *        or -1 if no minimum number is required.
   *
   * This method should be overridden to return a non-negative value if your
   * layer expects some minimum number of top blobs.
   */
  virtual inline int MinTopBlobs() const { return -1; }
  /**
   * @brief Returns the maximum number of top blobs required by the layer,
   *        or -1 if no maximum number is required.
   *
   * This method should be overridden to return a non-negative value if your
   * layer expects some maximum number of top blobs.
   */
  virtual inline int MaxTopBlobs() const { return -1; }
  /**
   * @brief Returns true if the layer requires an equal number of bottom and
   *        top blobs.
   *
   * This method should be overridden to return true if your layer expects an
   * equal number of bottom and top blobs.
   */
  virtual inline bool EqualNumBottomTopBlobs() const { return false; }

  /**
   * @brief Return whether "anonymous" top blobs are created automatically
   *        by the layer.
   *
   * If this method returns true, Net::Init will create enough "anonymous" top
   * blobs to fulfill the requirement specified by ExactNumTopBlobs() or
   * MinTopBlobs().
   */
  virtual inline bool AutoTopBlobs() const { return false; }

  /**
   * @brief Return whether to allow force_backward for a given bottom blob
   *        index.
   *
   * If AllowForceBackward(i) == false, we will ignore the force_backward
   * setting and backpropagate to blob i only if it needs gradient information
   * (as is done when force_backward == false).
   */
  virtual inline bool AllowForceBackward(const int bottom_index) const {
    return true;
```

# param_propagate_down 函数
判断是否需要计算梯度的一个函数
```
  /**
   * @brief Specifies whether the layer should compute gradients w.r.t. a
   *        parameter at a particular index given by param_id.
   *
   * You can safely ignore false values and always compute gradients
   * for all parameters, but possibly with wasteful computation.
   */
  inline bool param_propagate_down(const int param_id) {
    return (param_propagate_down_.size() > param_id) ?
        param_propagate_down_[param_id] : false;
  }
```
如果需要计算梯度的话就返回true，后面计算梯度的时候就每一个参数都去判断一下，如果需要就算一下梯度，如果不需要就不算。

# set_param_propagate_down 函数
设置是否需要返回梯度
```
  /**
   * @brief Sets whether the layer should compute gradients w.r.t. a
   *        parameter at a particular index given by param_id.
   */
  inline void set_param_propagate_down(const int param_id, const bool value) {
    if (param_propagate_down_.size() <= param_id) {
      param_propagate_down_.resize(param_id + 1, true);
    }
    param_propagate_down_[param_id] = value;
  }
```
上面那个函数是判断是否返回梯度，这个函数就是设置要不要返回。

# CheckBlobCounts 函数
检查bottom blob的各个数字是否和top blob匹配
```

  /**
   * Called by the parent Layer's SetUp to check that the number of bottom
   * and top Blobs provided as input match the expected numbers specified by
   * the {ExactNum,Min,Max}{Bottom,Top}Blobs() functions.
   */
  virtual void CheckBlobCounts(const vector<Blob<Dtype>*>& bottom,
                               const vector<Blob<Dtype>*>& top) {
    if (ExactNumBottomBlobs() >= 0) {
      CHECK_EQ(ExactNumBottomBlobs(), bottom.size())
          << type() << " Layer takes " << ExactNumBottomBlobs()
          << " bottom blob(s) as input.";
    }
    if (MinBottomBlobs() >= 0) {
      CHECK_LE(MinBottomBlobs(), bottom.size())
          << type() << " Layer takes at least " << MinBottomBlobs()
          << " bottom blob(s) as input.";
    }
    if (MaxBottomBlobs() >= 0) {
      CHECK_GE(MaxBottomBlobs(), bottom.size())
          << type() << " Layer takes at most " << MaxBottomBlobs()
          << " bottom blob(s) as input.";
    }
    if (ExactNumTopBlobs() >= 0) {
      CHECK_EQ(ExactNumTopBlobs(), top.size())
          << type() << " Layer produces " << ExactNumTopBlobs()
          << " top blob(s) as output.";
    }
    if (MinTopBlobs() >= 0) {
      CHECK_LE(MinTopBlobs(), top.size())
          << type() << " Layer produces at least " << MinTopBlobs()
          << " top blob(s) as output.";
    }
    if (MaxTopBlobs() >= 0) {
      CHECK_GE(MaxTopBlobs(), top.size())
          << type() << " Layer produces at most " << MaxTopBlobs()
          << " top blob(s) as output.";
    }
    if (EqualNumBottomTopBlobs()) {
      CHECK_EQ(bottom.size(), top.size())
          << type() << " Layer produces one top blob as output for each "
          << "bottom blob input.";
    }
  }
```

# SetLossWeights 函数
设置loss的权重，它是把计算出来的top diff更新进来，就是每次反向传播时候更新权重用的
```
  /**
   * Called by SetUp to initialize the weights associated with any top blobs in
   * the loss function. Store non-zero loss weights in the diff blob.
   */
  inline void SetLossWeights(const vector<Blob<Dtype>*>& top) {
    const int num_loss_weights = layer_param_.loss_weight_size();
    if (num_loss_weights) {
      CHECK_EQ(top.size(), num_loss_weights) << "loss_weight must be "
          "unspecified or specified once per top blob.";
      for (int top_id = 0; top_id < top.size(); ++top_id) {
        const Dtype loss_weight = layer_param_.loss_weight(top_id);
        if (loss_weight == Dtype(0)) { continue; }
        this->set_loss(top_id, loss_weight);
        const int count = top[top_id]->count();
        Dtype* loss_multiplier = top[top_id]->mutable_cpu_diff();
        caffe_set(count, loss_weight, loss_multiplier);
      }
    }
  }

```
可以看到它是从<code>top[top_id]->mutable_cpu_diff()</code>这里面读取的残差，这个就是caffe存中间结果的地方，读出来以后就更新到这一层就作为永久的权重了，直到下次更新改变。


# Forward 函数
layer这个类结束之后它也实现了基本的Forward函数
```
// Forward and backward wrappers. You should implement the cpu and
// gpu specific implementations instead, and should not change these
// functions.
```
这里注释也解释了一下，这里还是只打了个基本框架，你还是需要自己实现一个cpu或者gpu的实现来实现他的过程，但是他的这个基类函数千万不要动。
来看它这个函数
```
  Dtype loss = 0;
  Reshape(bottom, top);
  switch (Caffe::mode())
```
首先reshape了top的大小，但是为什么传入了bottom呢，因为top的大小跟bottom是无关的，但是缓存区的大小是根据bottom来计算的。就比如bottom大小是3，top是5，那么缓存区就应该是3*5=15.
```
  case Caffe::CPU:
    Forward_cpu(bottom, top);
    for (int top_id = 0; top_id < top.size(); ++top_id) {
      if (!this->loss(top_id)) { continue; }
      const int count = top[top_id]->count();
      const Dtype* data = top[top_id]->cpu_data();
      const Dtype* loss_weights = top[top_id]->cpu_diff();
      loss += caffe_cpu_dot(count, data, loss_weights);
    }
    break;
```
紧接着调用<code>Forward_cpu</code>这个函数，完了之后再把残差算出来。算的方法就是用这次新计算出来的残差点乘top层的data求和。
```
  case Caffe::GPU:
    Forward_gpu(bottom, top);
#ifndef CPU_ONLY
    for (int top_id = 0; top_id < top.size(); ++top_id) {
      if (!this->loss(top_id)) { continue; }
      const int count = top[top_id]->count();
      const Dtype* data = top[top_id]->gpu_data();
      const Dtype* loss_weights = top[top_id]->gpu_diff();
      Dtype blob_loss = 0;
      caffe_gpu_dot(count, data, loss_weights, &blob_loss);
      loss += blob_loss;
    }
#endif
    break;
  default:
    LOG(FATAL) << "Unknown caffe mode.";
  }
  return loss;
}
```
如果用了gpu的话也一样，只不过调用的是<code>Forward_gpu</code>这个函数，最后返回总的loss就可以了。

# Backward 函数
已经前传完之后还实现了反传函数
```
template <typename Dtype>
inline void Layer<Dtype>::Backward(const vector<Blob<Dtype>*>& top,
    const vector<bool>& propagate_down,
    const vector<Blob<Dtype>*>& bottom) {
  switch (Caffe::mode()) {
  case Caffe::CPU:
    Backward_cpu(top, propagate_down, bottom);
    break;
  case Caffe::GPU:
    Backward_gpu(top, propagate_down, bottom);
    break;
  default:
    LOG(FATAL) << "Unknown caffe mode.";
  }
}
```
这个反传函数也是一个大纲，它直接调用了<code>Backward_cpu</code>或者<code>Backward_gpu</code>就完了，因为反向传播每一层的差别太大了，所以他就实现的非常抽象了，全权交给子类来实现。

#  ToProto 函数
```
// Serialize LayerParameter to protocol buffer
template <typename Dtype>
void Layer<Dtype>::ToProto(LayerParameter* param, bool write_diff) {
  param->Clear();
  param->CopyFrom(layer_param_);
  param->clear_blobs();
  for (int i = 0; i < blobs_.size(); ++i) {
    blobs_[i]->ToProto(param->add_blobs(), write_diff);
  }
}
```
这个函数就是把blob写道protobuf里面