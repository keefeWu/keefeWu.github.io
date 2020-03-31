---
title: caffe源码阅读《六》base_conv_layer
date: 2020-03-05 20:46:35
tags: [caffe]
categories: 
- caffe源码解读
---
<code>BaseConvolutionLayer</code>是所有卷积层的基类。为什么卷积层还需要基类呢？因为再caffe里除了 <code>ConvolutionLayer</code>还有 <code>DeconvolutionLayer</code>
# BaseConvolutionLayer 类
## BaseConvolutionLayer 构造函数
```
  explicit BaseConvolutionLayer(const LayerParameter& param)
      : Layer<Dtype>(param) {}
```
因为它是一个基类，所以这个构造函数就写成了空的，传入了 <code>param</code>这一个参数，但是什么都没做，传一个参数把方向确定了，具体内容留给子类去实现。

## LayerSetUp 函数
这个 <code>LayerSetUp</code>函数是很关键的，因为它实现了，而且实现的非常长。
```
  virtual void LayerSetUp(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);
```
第一步首先加载我们这一个卷积的参数
```
  ConvolutionParameter conv_param = this->layer_param_.convolution_param();
```
这个参数是从 <code>layer_param_</code>这个变量里来的，而这个变量里的参数又是从哪里来的呢？这就要从 <code>Layer</code>这个类说起了，因为我们这个<code>BaseConvolutionLayer</code>是继承的<code>Layer</code>这个类。
```
template <typename Dtype>
class BaseConvolutionLayer : public Layer<Dtype>
```
所以一切的变量也是定义在<code>Layer</code>这个类里面的，我们去<code>Layer</code>这个类里面找一下。
```
  /** The protobuf that stores the layer parameters */
  LayerParameter layer_param_;
```
它定义的这个 <code>LayerParameter</code>是在 <code>proto</code>文件里
```
class LayerParameter : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:caffe.LayerParameter) */ 
```
而 <code>convolution_param</code>函数把参数变量的指针返回回来，实际上就是把参数返回到了这个类里面。
```
inline const ::caffe::ConvolutionParameter& LayerParameter::convolution_param() const {
  // @@protoc_insertion_point(field_get:caffe.LayerParameter.convolution_param)
  return convolution_param_ != NULL ? *convolution_param_ : *default_instance_->convolution_param_;
}
```
接着问一下是不是要强制把矩阵转成caffe的矩阵形式，就是我们说的 <code>im2col</code>
```
  force_nd_im2col_ = conv_param.force_nd_im2col();
```
这个 <code>im2col</code>真的非常重要，我觉得这个是caffe计算部分的核心，建议大家一定要认真的学习这一块。
接着获取bottom的 <code>channel_axis</code>
```
  channel_axis_ = bottom[0]->CanonicalAxisIndex(conv_param.axis());
```
这一步不是获取 <code>channel</code>的数量，而是返回 <code>channel</code>在 <code>shape</code>里是第几位，比方说 <code>shape</code>是 <code>3*224*224</code>,那么 <code>channel</code>就在第0位，这一步返回值就是0.
所以真正空间形状的开始就是从第1位开始的，接下来这个函数就是返回的空间形状开始的那个axis
```
  const int first_spatial_axis = channel_axis_ + 1;
```
然后是总共的axis的数量
```
  const int num_axes = bottom[0]->num_axes();
```
接下来是 <code>num_spatial_axes_</code>,这个代表的就是真正空间的axis的数量，也就是排除了channel那一层的数量
```
  num_spatial_axes_ = num_axes - first_spatial_axis;
```
检查一下 <code>num_spatial_axes_</code>是不是大于0的
```
  CHECK_GE(num_spatial_axes_, 0);
```
因为如果不是大于0的就说明空间的维度不存在，这显然是不可能的，就会直接抛出异常。
然后声明了一个变量，专门去存空间的形状。
```
  vector<int> spatial_dim_blob_shape(1, std::max(num_spatial_axes_, 1));
```
所以刚才才要了空间形状的起始位置，因为在这里等着，准备拷贝空间形状。
因为卷积核的空间维度数量要和图片是一样的，所以要把卷积核 <code>Reshape</code>了
```
  // Setup filter kernel dimensions (kernel_shape_).
  kernel_shape_.Reshape(spatial_dim_blob_shape);
```
注意啊，这里是维度是一样的，并不是形状是一样的，也就是说假设图片是个2维图片，那么卷积核也就必须是2维的。假设图片大小是224*224，那么卷积核是3*3就可以和它匹配。假设图片是三维的，比方说224*224*224，那么卷积核也就必须是三维的，比方说5*5*5.
接着准备对 <code>kernel_shape_</code>赋值了
```
int* kernel_shape_data = kernel_shape_.mutable_cpu_data();
```
我们知道 <code>mutable_cpu_data</code>存的是最新修改的数据的指针，所以这里取了这个地址，就准备开始赋值。
```
  if (conv_param.has_kernel_h() || conv_param.has_kernel_w()) {
    CHECK_EQ(num_spatial_axes_, 2)
        << "kernel_h & kernel_w can only be used for 2D convolution.";
    CHECK_EQ(0, conv_param.kernel_size_size())
        << "Either kernel_size or kernel_h/w should be specified; not both.";
    kernel_shape_data[0] = conv_param.kernel_h();
    kernel_shape_data[1] = conv_param.kernel_w();
  } else {
    const int num_kernel_dims = conv_param.kernel_size_size();
    CHECK(num_kernel_dims == 1 || num_kernel_dims == num_spatial_axes_)
        << "kernel_size must be specified once, or once per spatial dimension "
        << "(kernel_size specified " << num_kernel_dims << " times; "
        << num_spatial_axes_ << " spatial dims).";
      for (int i = 0; i < num_spatial_axes_; ++i) {
        kernel_shape_data[i] =
            conv_param.kernel_size((num_kernel_dims == 1) ? 0 : i);
      }
  }
```
这里判断了，如果是二维的话，那么就从参数里面读取高和宽，如果是多维的话，就一维一维的从参数里面去读取。
最后检查一下空间的维度有没有存在0
```
  for (int i = 0; i < num_spatial_axes_; ++i) {
    CHECK_GT(kernel_shape_data[i], 0) << "Filter dimensions must be nonzero.";
  }
```
这一步很好理解，你总不能有高或者宽是0的图吧
接下来把 <code>stride_</code>这个blob也给它 <code>Reshape</code>一下，因为这个 <code>stride_</code> blob它是记录步长的一个blob，所以它的维度也得和我们这个图片是一致的。
```
  // Setup stride dimensions (stride_).
  stride_.Reshape(spatial_dim_blob_shape);
  int* stride_data = stride_.mutable_cpu_data();
  if (conv_param.has_stride_h() || conv_param.has_stride_w()) {
    CHECK_EQ(num_spatial_axes_, 2)
        << "stride_h & stride_w can only be used for 2D convolution.";
    CHECK_EQ(0, conv_param.stride_size())
        << "Either stride or stride_h/w should be specified; not both.";
    stride_data[0] = conv_param.stride_h();
    stride_data[1] = conv_param.stride_w();
  } else {
    const int num_stride_dims = conv_param.stride_size();
    CHECK(num_stride_dims == 0 || num_stride_dims == 1 ||
          num_stride_dims == num_spatial_axes_)
        << "stride must be specified once, or once per spatial dimension "
        << "(stride specified " << num_stride_dims << " times; "
        << num_spatial_axes_ << " spatial dims).";
    const int kDefaultStride = 1;
    for (int i = 0; i < num_spatial_axes_; ++i) {
      stride_data[i] = (num_stride_dims == 0) ? kDefaultStride :
          conv_param.stride((num_stride_dims == 1) ? 0 : i);
      CHECK_GT(stride_data[i], 0) << "Stride dimensions must be nonzero.";
    }
  }
```
和上面的卷积核是一样的。
然后是 <code>pad_</code>,和 <code>stride_</code>一模一样，也是给他形状赋一个值。padding就是卷积边界处理的长度。
```
  // Setup pad dimensions (pad_).
  pad_.Reshape(spatial_dim_blob_shape);
  int* pad_data = pad_.mutable_cpu_data();
  if (conv_param.has_pad_h() || conv_param.has_pad_w()) {
    CHECK_EQ(num_spatial_axes_, 2)
        << "pad_h & pad_w can only be used for 2D convolution.";
    CHECK_EQ(0, conv_param.pad_size())
        << "Either pad or pad_h/w should be specified; not both.";
    pad_data[0] = conv_param.pad_h();
    pad_data[1] = conv_param.pad_w();
  } else {
    const int num_pad_dims = conv_param.pad_size();
    CHECK(num_pad_dims == 0 || num_pad_dims == 1 ||
          num_pad_dims == num_spatial_axes_)
        << "pad must be specified once, or once per spatial dimension "
        << "(pad specified " << num_pad_dims << " times; "
        << num_spatial_axes_ << " spatial dims).";
    const int kDefaultPad = 0;
    for (int i = 0; i < num_spatial_axes_; ++i) {
      pad_data[i] = (num_pad_dims == 0) ? kDefaultPad :
          conv_param.pad((num_pad_dims == 1) ? 0 : i);
    }
  }
```
还有决定这个图将被放大多少倍的 <code>dilation_</code>也是一样的。
```
  // Setup dilation dimensions (dilation_).
  dilation_.Reshape(spatial_dim_blob_shape);
  int* dilation_data = dilation_.mutable_cpu_data();
  const int num_dilation_dims = conv_param.dilation_size();
  CHECK(num_dilation_dims == 0 || num_dilation_dims == 1 ||
        num_dilation_dims == num_spatial_axes_)
      << "dilation must be specified once, or once per spatial dimension "
      << "(dilation specified " << num_dilation_dims << " times; "
      << num_spatial_axes_ << " spatial dims).";
  const int kDefaultDilation = 1;
  for (int i = 0; i < num_spatial_axes_; ++i) {
    dilation_data[i] = (num_dilation_dims == 0) ? kDefaultDilation :
                       conv_param.dilation((num_dilation_dims == 1) ? 0 : i);
  }
```
接着就要开始转成计算矩阵了。
首先判断一下图是不是1*1的，因为如果是1*1的话，那么这个计算就会省很多事情。
```
  // Special case: im2col is the identity for 1x1 convolution with stride 1
  // and no padding, so flag for skipping the buffer and transformation.
  is_1x1_ = true;
  for (int i = 0; i < num_spatial_axes_; ++i) {
    is_1x1_ &=
        kernel_shape_data[i] == 1 && stride_data[i] == 1 && pad_data[i] == 0;
    if (!is_1x1_) { break; }
  }
```
接着获取 <code>bottom</code>层的通道数，也就是输入的通道数，还有 <code>output</code>的数量，也就是输出的通道数,还有 <code>group</code>的数量，分 <code>group</code>的目的是为了指定哪些通道只能哪些卷积核做卷积，不相干的就不会卷积。
```
  // Configure output channels and groups.
  channels_ = bottom[0]->shape(channel_axis_);
  num_output_ = this->layer_param_.convolution_param().num_output();
  CHECK_GT(num_output_, 0);
  group_ = this->layer_param_.convolution_param().group();
  CHECK_EQ(channels_ % group_, 0);
  CHECK_EQ(num_output_ % group_, 0)
      << "Number of output should be multiples of group.";
```
因为分组是均分的，所以判断了一下 <code>group</code>的数量是不是能被 <code>channel</code>的数量整除。
之后判断一下输入和输出顺序是不是设定了需要反过来。
```
  if (reverse_dimensions()) {
    conv_out_channels_ = channels_;
    conv_in_channels_ = num_output_;
  } else {
    conv_out_channels_ = num_output_;
    conv_in_channels_ = channels_;
  }
```
如果是反卷积这种的话就要把输入和输出的channel给反过来了。
定义一下权值的形状
```
  // Handle the parameters: weights and biases.
  // - blobs_[0] holds the filter weights
  // - blobs_[1] holds the biases (optional)
  vector<int> weight_shape(2);
  weight_shape[0] = conv_out_channels_;
  weight_shape[1] = conv_in_channels_ / group_;
  for (int i = 0; i < num_spatial_axes_; ++i) {
    weight_shape.push_back(kernel_shape_data[i]);
  }
```
然后是 <code>bias</code>,因为这个 <code>bias</code>是可以选择开关的，所以首先判断了一下，如果开的话就定义一下这个偏置的形状.
```
  bias_term_ = this->layer_param_.convolution_param().bias_term();
  vector<int> bias_shape(bias_term_, num_output_);
```
最后检查 <code>blob</code>的 <code>size</code>
```
  if (this->blobs_.size() > 0) {
    CHECK_EQ(1 + bias_term_, this->blobs_.size())
        << "Incorrect number of weight blobs.";
    if (weight_shape != this->blobs_[0]->shape()) {
      Blob<Dtype> weight_shaped_blob(weight_shape);
      LOG(FATAL) << "Incorrect weight shape: expected shape "
          << weight_shaped_blob.shape_string() << "; instead, shape was "
          << this->blobs_[0]->shape_string();
    }
    if (bias_term_ && bias_shape != this->blobs_[1]->shape()) {
      Blob<Dtype> bias_shaped_blob(bias_shape);
      LOG(FATAL) << "Incorrect bias shape: expected shape "
          << bias_shaped_blob.shape_string() << "; instead, shape was "
          << this->blobs_[1]->shape_string();
    }
    LOG(INFO) << "Skipping parameter initialization";
  } else {
    if (bias_term_) {
      this->blobs_.resize(2);
    } else {
      this->blobs_.resize(1);
    }
```
如果没有开 <code>bias</code>的话，那么 <code>blob</code>的size就是1，只存了权值，如果开了的话就是2，所以它和这个 <code>1+bias_term</code>做比较看看是否相等。其实这样写是个非常差劲的写法，很具有迷惑性，但是结果肯定是对的，因为布尔变量为true就是1.
然后初始化权值和bias
```
    // Initialize and fill the weights:
    // output channels x input channels per-group x kernel height x kernel width
    this->blobs_[0].reset(new Blob<Dtype>(weight_shape));
    shared_ptr<Filler<Dtype> > weight_filler(GetFiller<Dtype>(
        this->layer_param_.convolution_param().weight_filler()));
    weight_filler->Fill(this->blobs_[0].get());
    // If necessary, initialize and fill the biases.
    if (bias_term_) {
      this->blobs_[1].reset(new Blob<Dtype>(bias_shape));
      shared_ptr<Filler<Dtype> > bias_filler(GetFiller<Dtype>(
          this->layer_param_.convolution_param().bias_filler()));
      bias_filler->Fill(this->blobs_[1].get());
    }
  }
```
从文件中读取权值，然后填给blob里面。
接着获取一下卷积核的总大小
```
  kernel_dim_ = this->blobs_[0]->count(1);
```
这个 <code>count</code>就是总的大小，比方说卷积核形状是3*5*5，那么 <code>count</code>的结果就是75，而加了个参数就是从第几位开始，那么 <code>count(1)</code>就是5*5=25
因为第0位就是输入的channel，这个和卷积核没什么关系，所以从1开始数，用来分配矩阵的大小。
接着根据分组情况找一下权值的指针偏移量，方便换组的时候知道往后数多少位
```
  weight_offset_ = conv_out_channels_ * kernel_dim_ / group_;
```
最后在 <code>blob</code>里给反向传播腾个位置就结束了
```
  // Propagate gradients to the parameters (as directed by backward pass).
  this->param_propagate_down_.resize(this->blobs_.size(), true);
```

## Reshape 函数
```
  virtual void Reshape(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);
```
<code>Reshape</code>函数传入了两个参数，一个 <code>bottom</code>一个 <code>top</code>。
一开始还是获取了 <code>first_spatial_axis</code>
```
  const int first_spatial_axis = channel_axis_ + 1;
```
然后开始检查 <code>spatial_axis</code>和 <code>bottom</code>的<code>num_axes</code>是不是一致的。
```
  CHECK_EQ(bottom[0]->num_axes(), first_spatial_axis + num_spatial_axes_)
      << "bottom num_axes may not change.";
```