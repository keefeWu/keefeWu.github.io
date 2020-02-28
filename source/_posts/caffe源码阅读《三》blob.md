---
title: caffe源码阅读《三》blob
date: 2020-02-26 09:51:07
tags: [caffe]
categories: 
- caffe源码解读
---

今天来阅读的是caffe的<code>blob</code>部分，这个<code>blob</code>它是有cpp的，因为这个<code>blob</code>是caffe核心的数据存储结构，后面所有的操作都会用到这个地方的，所以这里是caffe官方来实现的。
首先看hpp文件。
他是有个<code>Blob</code>的类
```
/**
 * @brief A wrapper around SyncedMemory holders serving as the basic
 *        computational unit through which Layer%s, Net%s, and Solver%s
 *        interact.
 *
 * TODO(dox): more thorough description.
 */
template <typename Dtype>
class Blob 
```
这里注释写了通过这个<code>SyncedMemory</code>撑起来整个计算单元的服务，把<code>Layer</code>,<code>Net</code>还有<code>Solver</code>连接起来的一个核心的类。
这个<code>SyncedMemory</code>是什么呢，有兴趣的可以去了解一下<code>自动状态转换机</code>，因为caffe分为cpu版和gpu版，他需要时刻知道数据现在放在哪里，他就用了一个叫<code>自动状态转换机</code>的方法。
简单介绍一下，如果数据在cpu里，那么他会有一个状态叫做<code>cpu_head</code>，有一个函数叫做 <code>to_cpu</code> ，用来把数据存到cpu,如果在gpu里就换成gpu。
接下来直接看代码
# Blob 构造函数
它的构造函数有三种
```
  Blob()
       : data_(), diff_(), count_(0), capacity_(0) {}
```
第一个是直接给这四个变量赋值，<code>data_ diff_ count_ capacity_</code>这4个家伙不是函数，是变量，他在下面有定义。
```
  shared_ptr<SyncedMemory> data_;
  shared_ptr<SyncedMemory> diff_;
  int count_;
  int capacity_;
```  
分别代表数据，梯度，已存储的参数个数，最大可存储的参数个数。
所以这个构造函数就是初始化了这四个值。
```
  /// @brief Deprecated; use <code>Blob(const vector<int>& shape)</code>.
  explicit Blob(const int num, const int channels, const int height,
      const int width);
```
这个显示的构造函数是传入的 <code>num channels height width</code>,它也是直接初始化了blob的这4个值
```
  explicit Blob(const vector<int>& shape);
```
最后这个构造函数和第二个其实是一样的，只不过它是通过做成了 <code>vector</code>变量来解决的。
这里 <code>num</code>就是指的传入的大小，<code>channels</code>是输出的大小，<code>width</code>和 <code>height</code>就是卷积的大小了。

# Reshape 函数
在调用构造函数传入大小之后就开始调用 <code>Reshape</code>函数了。
```
template <typename Dtype>
void Blob<Dtype>::Reshape(const int num, const int channels, const int height,
    const int width) {
  vector<int> shape(4);
  shape[0] = num;
  shape[1] = channels;
  shape[2] = height;
  shape[3] = width;
  Reshape(shape);
}
```
这个 <code>Reshape</code>函数调用了另一个调用了 <code>vector</code>的 <code>Reshape</code>函数。
```
template <typename Dtype>
void Blob<Dtype>::Reshape(const vector<int>& shape)
```
来看看它的详细步骤
```
  CHECK_LE(shape.size(), kMaxBlobAxes);
  count_ = 1;
  shape_.resize(shape.size());
```
它首先先把自己的私有成员变量 <code>shape_</code>resize成了传进来的那个 <code>shape</code>的size。
它的这个 <code>shape</code>的size指的是blob的维度，不是卷积核的大小，blob的维度默认就是<code>num channels height width</code>这4个维度，所以默认的 <code>shape.size()</code>就是4，<code>shape</code>是一个vector数组，它的内容是每个通道的大小，但是它的size是4，当然，用户也是可以自定义它的size的，适用于特殊情况，所以在上面才做了一个判断 <code>CHECK_LE(shape.size(), kMaxBlobAxes);</code>来确认它的size是一定小于等于用户给定的大小的。
```
  if (!shape_data_ || shape_data_->size() < shape.size() * sizeof(int)) {
    shape_data_.reset(new SyncedMemory(shape.size() * sizeof(int)));
  }
  int* shape_data = static_cast<int*>(shape_data_->mutable_cpu_data());
```
如果这个 <code>shape_data_</code>是空的或者它的size小于传进来的size，那么就重新给他分配空间，最后又定义了一个指针，指向了 <code>shape_data_->mutable_cpu_data()</code>
```
  for (int i = 0; i < shape.size(); ++i) {
    CHECK_GE(shape[i], 0);
    if (count_ != 0) {
      CHECK_LE(shape[i], INT_MAX / count_) << "blob size exceeds INT_MAX";
    }
    count_ *= shape[i];
    shape_[i] = shape[i];
    shape_data[i] = shape[i];
  }
```
然后检查了一下有没有是0的shape，或者有没有超过了int范围的shape，如果都没有就开始给他赋值了。解释一下count，count的定义本身就是每个shape的积，比如说width*height*num*channels，所以这里初始化一个1，然后就让他*=shape[i]
```
  if (count_ > capacity_) {
    capacity_ = count_;
    data_.reset(new SyncedMemory(capacity_ * sizeof(Dtype)));
    diff_.reset(new SyncedMemory(capacity_ * sizeof(Dtype)));
  }
```
如果count>容积的话，我们就要把容积更新，然后把这个data按照容积重新reset一下。

# shape_string函数
这里有个 <code>shape_string</code>函数，他其实是把shape做成一个string，用来打印的
```
  inline string shape_string() const {
    ostringstream stream;
    for (int i = 0; i < shape_.size(); ++i) {
      stream << shape_[i] << " ";
    }
    stream << "(" << count_ << ")";
    return stream.str();
  }
```
可以看到它的代码，其实就是遍历了shape，然后把每一个值拼接起来，方便打印调试用的。

# shape 函数
这个 <code>shape</code>函数有两个，它其实就是返回出成员变量 <code>shape_</code>,
```
  inline const vector<int>& shape() const { return shape_; }
  /**
   * @brief Returns the dimension of the index-th axis (or the negative index-th
   *        axis from the end, if index is negative).
   *
   * @param index the axis index, which may be negative as it will be
   *        "canonicalized" using CanonicalAxisIndex.
   *        Dies on out of range index.
   */
  inline int shape(int index) const {
    return shape_[CanonicalAxisIndex(index)];
  }
```
第二个带参数的这个它的参数是坐标轴，就是如果带有轴的话，那么就只返回那个index下的shape。所以它是int类型
这里有个小彩蛋，它为什么不直接使用 <code>shape_[index]</code>而是使用 <code>shape_[CanonicalAxisIndex(index)]</code>呢？
我们看看这个函数
```

  /**
   * @brief Returns the 'canonical' version of a (usually) user-specified axis,
   *        allowing for negative indexing (e.g., -1 for the last axis).
   *
   * @param axis_index the axis index.
   *        If 0 <= index < num_axes(), return index.
   *        If -num_axes <= index <= -1, return (num_axes() - (-index)),
   *        e.g., the last axis index (num_axes() - 1) if index == -1,
   *        the second to last if index == -2, etc.
   *        Dies on out of range index.
   */
  inline int CanonicalAxisIndex(int axis_index) const {
    CHECK_GE(axis_index, -num_axes())
        << "axis " << axis_index << " out of range for " << num_axes()
        << "-D Blob with shape " << shape_string();
    CHECK_LT(axis_index, num_axes())
        << "axis " << axis_index << " out of range for " << num_axes()
        << "-D Blob with shape " << shape_string();
    if (axis_index < 0) {
      return axis_index + num_axes();
    }
    return axis_index;
  }
```
它为了防止用户乱输入index，做了很多判断，如果index小于0他会加上num_axes，这样就相当于倒过来数也是可以的，就和python里的数组一样了，-1就是最后一个。

# num_axes 函数
返回了shape的size
```
  inline int num_axes() const { return shape_.size(); }
```

# count 函数
这个 <code>count</code>函数有3个
```
  inline int count() const { return count_; }
```
这个没有参数的直接就返回了成员变量 <code>count_</code>

```
  /**
   * @brief Compute the volume of a slice; i.e., the product of dimensions
   *        among a range of axes.
   *
   * @param start_axis The first axis to include in the slice.
   *
   * @param end_axis The first axis to exclude from the slice.
   */
  inline int count(int start_axis, int end_axis) const {
    CHECK_LE(start_axis, end_axis);
    CHECK_GE(start_axis, 0);
    CHECK_GE(end_axis, 0);
    CHECK_LE(start_axis, num_axes());
    CHECK_LE(end_axis, num_axes());
    int count = 1;
    for (int i = start_axis; i < end_axis; ++i) {
      count *= shape(i);
    }
    return count;
  }
```
而这个带了 <code>start_axis</code>和 <code>end_axis</code>的则有了起点和中i的那，可以看到，它的循环都有了起始，重新给计算了一遍。
还有一个
```
  /**
   * @brief Compute the volume of a slice spanning from a particular first
   *        axis to the final axis.
   *
   * @param start_axis The first axis to include in the slice.
   */
  inline int count(int start_axis) const {
    return count(start_axis, num_axes());
  }
```
这个只带了起点，终点就是最后一个shape了。

# offset 函数
判断给定的这个元素它的偏移量是多少
```
  inline int offset(const int n, const int c = 0, const int h = 0,
      const int w = 0) const {
    CHECK_GE(n, 0);
    CHECK_LE(n, num());
    CHECK_GE(channels(), 0);
    CHECK_LE(c, channels());
    CHECK_GE(height(), 0);
    CHECK_LE(h, height());
    CHECK_GE(width(), 0);
    CHECK_LE(w, width());
    return ((n * channels() + c) * height() + h) * width() + w;
  }
```
比方说我们要查询num是1，channel是2，height是3，width是4的这个元素它在在内存中或者数组中的下表是多少，就直接调用这个函数计算一遍就可以了，这个做多维数组处理的时候经常会用到。
```
  inline int offset(const vector<int>& indices) const {
    CHECK_LE(indices.size(), num_axes());
    int offset = 0;
    for (int i = 0; i < num_axes(); ++i) {
      offset *= shape(i);
      if (indices.size() > i) {
        CHECK_GE(indices[i], 0);
        CHECK_LT(indices[i], shape(i));
        offset += indices[i];
      }
    }
    return offset;
  }
```
他还有另外这个函数，只是把之前分开的参数改成了一个vector，实际上和刚才是一样的。

# CopyFrom 函数
这个函数是从设备上计算出来的结果拷贝到这个 blob里面的一个函数
```
template <typename Dtype>
void Blob<Dtype>::CopyFrom(const Blob& source, bool copy_diff, bool reshape) {
  if (source.count() != count_ || source.shape() != shape_) {
    if (reshape) {
      ReshapeLike(source);
    } else {
      LOG(FATAL) << "Trying to copy blobs of different sizes.";
    }
  }
  switch (Caffe::mode()) {
  case Caffe::GPU:
    if (copy_diff) {
      caffe_copy(count_, source.gpu_diff(),
          static_cast<Dtype*>(diff_->mutable_gpu_data()));
    } else {
      caffe_copy(count_, source.gpu_data(),
          static_cast<Dtype*>(data_->mutable_gpu_data()));
    }
    break;
  case Caffe::CPU:
    if (copy_diff) {
      caffe_copy(count_, source.cpu_diff(),
          static_cast<Dtype*>(diff_->mutable_cpu_data()));
    } else {
      caffe_copy(count_, source.cpu_data(),
          static_cast<Dtype*>(data_->mutable_cpu_data()));
    }
    break;
  default:
    LOG(FATAL) << "Unknown caffe mode.";
  }
}
```
可以看到，如果是cpu则从cpu拷贝，如果是gpu则从gpu拷贝。
它的第二个参数 <code>copy_diff</code>如果是 <code>true</code>的话就带上diff，如果是 <code>false</code>的话就只拷贝data
# data_at 和 diff_at 函数
这两个函数做的事情是类似的
```

  inline Dtype data_at(const int n, const int c, const int h,
      const int w) const {
    return cpu_data()[offset(n, c, h, w)];
  }

  inline Dtype diff_at(const int n, const int c, const int h,
      const int w) const {
    return cpu_diff()[offset(n, c, h, w)];
  }

  inline Dtype data_at(const vector<int>& index) const {
    return cpu_data()[offset(index)];
  }

  inline Dtype diff_at(const vector<int>& index) const {
    return cpu_diff()[offset(index)];
  }
```
就是传入一个坐标，然后返回这个坐标的值

