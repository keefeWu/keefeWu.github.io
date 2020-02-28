---
title: caffe源码阅读《四》net
date: 2020-02-28 15:27:32
tags: [caffe]
categories: 
- caffe源码解读
---

caffe的net是一个网络架构，串联起所有的blob，支撑着整个神经网络前向和反向传播的一个结构。他也是分为hpp和cpp文件的。
首先看注释
```
/**
 * @brief Connects Layer%s together into a directed acyclic graph (DAG)
 *        specified by a NetParameter.
 *
 * TODO(dox): more thorough description.
 */
template <typename Dtype>
class Net
```
把各个层连接在一起，形成一个有向无环图。
接着直接看代码

# Net 构造函数
```
  explicit Net(const NetParameter& param);
  explicit Net(const string& param_file, Phase phase,
      const int level = 0, const vector<string>* stages = NULL);
```
首先是构造函数，他又两个，第一个传入的是 <code>NetParameter</code>的对象，第二个是通过文件读取的，传入的文件路径和一个 <code>phase</code>, <code>phase</code>是指的我这个网络的状态是 <code>train</code> 还是 <code>test</code>的，然后还有 <code>level</code> 和 <code>stages</code>.
看看它的具体实现
```
template <typename Dtype>
Net<Dtype>::Net(const string& param_file, Phase phase,
    const int level, const vector<string>* stages) {
  NetParameter param;
  ReadNetParamsFromTextFileOrDie(param_file, &param);
  // Set phase, stages and level
  param.mutable_state()->set_phase(phase);
  if (stages != NULL) {
    for (int i = 0; i < stages->size(); i++) {
      param.mutable_state()->add_stage((*stages)[i]);
    }
  }
  param.mutable_state()->set_level(level);
  Init(param);
}
```
由于我们参数是从文件读取的，所以这个
```
NetParameter param;
```
是我们自己定义的。
然后是执行了
```
ReadNetParamsFromTextFileOrDie(param_file, &param);
```
这个函数传入了两个参数，一个是这个文件的地址，一个是 <code>param</code>这个变量，用来接收文件中读取的配置参数。这个函数是位于 <code>src/caffe/util/upgrade_proto.cpp</code>这个目录下的，它的大概功能就是从这个文件里面读取参数到 <code>param</code>这个变量里去。这个 <code>param</code>的格式是在 <code>src/caffe/proto/caffe.proto</code>里面定义的，我们可以去看一看。
```
message NetParameter {
  optional string name = 1; // consider giving the network a name
  // DEPRECATED. See InputParameter. The input blobs to the network.
  repeated string input = 3;
  // DEPRECATED. See InputParameter. The shape of the input blobs.
  repeated BlobShape input_shape = 8;

  // 4D input dimensions -- deprecated.  Use "input_shape" instead.
  // If specified, for each input blob there should be four
  // values specifying the num, channels, height and width of the input blob.
  // Thus, there should be a total of (4 * #input) numbers.
  repeated int32 input_dim = 4;

  // Whether the network will force every layer to carry out backward operation.
  // If set False, then whether to carry out backward is determined
  // automatically according to the net structure and learning rates.
  optional bool force_backward = 5 [default = false];
  // The current "state" of the network, including the phase, level, and stage.
  // Some layers may be included/excluded depending on this state and the states
  // specified in the layers' include and exclude fields.
  optional NetState state = 6;

  // Print debugging information about results while running Net::Forward,
  // Net::Backward, and Net::Update.
  optional bool debug_info = 7 [default = false];

  // The layers that make up the net.  Each of their configurations, including
  // connectivity and behavior, is specified as a LayerParameter.
  repeated LayerParameter layer = 100;  // ID 100 so layers are printed last.

  // DEPRECATED: use 'layer' instead.
  repeated V1LayerParameter layers = 2;
}
```
这里面还是定义了很多东西的，并且都有非常详细的注释，比方说网络的名字，形状，是否打印等等。
紧接着是设置状态，就是下面这一步。
```
param.mutable_state()->set_phase(phase);
```
然后如果设置了 <code>stage</code>的话，就把每一层的 <code>stage</code>加载进来。
然后 <code>level</code>也是同理
```
param.mutable_state()->set_level(level);
```
最后初始化一下 <code>param</code>就完了
```
Init(param);
```

然后我们再看直接传入这个 <code>NetParameter</code>的构造函数
```
template <typename Dtype>
Net<Dtype>::Net(const NetParameter& param) {
  Init(param);
}
```
他就是直接调用了 <code>Init</code>这个函数。
然后我们再看一下 <code>Init</code>这个函数，它也是在这个 <code>Net</code>类里面。

# Init 函数
```
  /// @brief Initialize a network with a NetParameter.
  void Init(const NetParameter& param);
```
这个函数实际上就是做一个初始化的功能。
```
  // Set phase from the state.
  phase_ = in_param.state().phase();
```
比方这一句就是把从参数里读取的 <code>phase</code>赋值给自己的私有变量 <code>phase_</code>.
```
  // Filter layers based on their include/exclude rules and
  // the current NetState.
  NetParameter filtered_param;
  FilterNet(in_param, &filtered_param);
  LOG_IF(INFO, Caffe::root_solver())
      << "Initializing net from parameters: " << std::endl
      << filtered_param.DebugString();
```
然后定义了一个 <code>NetParameter</code>的变量，去调用了一个 <code>FilterNet</code>函数， 这个<code>FilterNet</code>函数是干嘛的呢？
它是根据 <code>state</code>初始化所有层的一个函数。这个后面紧跟着就会讲解。
```
  // Create a copy of filtered_param with splits added where necessary.
  NetParameter param;
  InsertSplits(filtered_param, &param);
```
然后跟着的是 <code>InsertSplits</code>这个函数，这个函数是干嘛的呢？
因为有的层可能会分裂，下面会有两条路可以走，比方说有两个top，那么这个时候就需要分割出来了。
```
  // For each layer, set up its input and output
  bottom_vecs_.resize(param.layer_size());
  top_vecs_.resize(param.layer_size());
  bottom_id_vecs_.resize(param.layer_size());
  param_id_vecs_.resize(param.layer_size());
  top_id_vecs_.resize(param.layer_size());
  bottom_need_backward_.resize(param.layer_size());
```
接下来的这个是根据layer的大小给这些vector分配空间
```
    if (!param.layer(layer_id).has_phase()) {
      param.mutable_layer(layer_id)->set_phase(phase_);
    }
```
给每个layer分配 <code>phase</code>，决定它是 <code>train</code>还是 <code>test</code>
```
    layers_.push_back(LayerRegistry<Dtype>::CreateLayer(layer_param));
    layer_names_.push_back(layer_param.name());
    LOG_IF(INFO, Caffe::root_solver())
        << "Creating Layer " << layer_param.name();
    bool need_backward = false;

    // Figure out this layer's input and output
    for (int bottom_id = 0; bottom_id < layer_param.bottom_size();
         ++bottom_id) {
      const int blob_id = AppendBottom(param, layer_id, bottom_id,
                                       &available_blobs, &blob_name_to_idx);
      // If a blob needs backward, this layer should provide it.
      need_backward |= blob_need_backward_[blob_id];
    }
```
然后开始把每个layer链接起来，因为它是自 <code>top</code>向 <code>bottom</code>链接的，所以这里是 <code>AppendBottom</code>，他的 <code>bottom</code>添加了对应的blob，也就是相当于一个链表一样的结构，每个 <code>bottom</code>添加了后面一个  <code>bottom</code>的id。

```
    int num_top = layer_param.top_size();
    for (int top_id = 0; top_id < num_top; ++top_id) {
      AppendTop(param, layer_id, top_id, &available_blobs, &blob_name_to_idx);
      // Collect Input layer tops as Net inputs.
      if (layer_param.type() == "Input") {
        const int blob_id = blobs_.size() - 1;
        net_input_blob_indices_.push_back(blob_id);
        net_input_blobs_.push_back(blobs_[blob_id].get());
      }
    }
```
然后是把 <code>top</code>对应层的id添加进去，也就是相当于一个双向的链表一样的。
```
    Layer<Dtype>* layer = layers_[layer_id].get();
    if (layer->AutoTopBlobs()) {
      const int needed_num_top =
          std::max(layer->MinTopBlobs(), layer->ExactNumTopBlobs());
      for (; num_top < needed_num_top; ++num_top) {
        // Add "anonymous" top blobs -- do not modify available_blobs or
        // blob_name_to_idx as we don't want these blobs to be usable as input
        // to other layers.
        AppendTop(param, layer_id, num_top, NULL, NULL);
      }
    }
```
如果你这个层定义了自动top的话，那么这里就会把自动top添加进去。
这个自动top blob就是之前我们在layer看到的这么一个接口。至于每个层具体怎么实现的其实是在不同层有不同的实现方法的。
```
    // After this layer is connected, set it up.
    layers_[layer_id]->SetUp(bottom_vecs_[layer_id], top_vecs_[layer_id]);
```
全部都添加好了以后就开始走这个层的 <code>setup</code>函数，这个 <code>setup</code>我们之前在 <code>layer</code>里面讲到过，就是那4步，就是。
```
  void SetUp(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top) {
    CheckBlobCounts(bottom, top);
    LayerSetUp(bottom, top);
    Reshape(bottom, top);
    SetLossWeights(top);
  }
```
* 检查blob的count
* 走layer的setup
* 然后reshape它
* 最后设置loss权重
```
    for (int top_id = 0; top_id < top_vecs_[layer_id].size(); ++top_id) {
      if (blob_loss_weights_.size() <= top_id_vecs_[layer_id][top_id]) {
        blob_loss_weights_.resize(top_id_vecs_[layer_id][top_id] + 1, Dtype(0));
      }
      blob_loss_weights_[top_id_vecs_[layer_id][top_id]] = layer->loss(top_id);
      LOG_IF(INFO, Caffe::root_solver())
          << "Top shape: " << top_vecs_[layer_id][top_id]->shape_string();
      if (layer->loss(top_id)) {
        LOG_IF(INFO, Caffe::root_solver())
            << "    with loss weight " << layer->loss(top_id);
      }
      memory_used_ += top_vecs_[layer_id][top_id]->count();
    }
```
然后resize一下loss权重的大小，最后还统计了一下会用到多少内存，然后打印出来。
```
    for (int param_id = 0; param_id < num_param_blobs; ++param_id) {
      const ParamSpec* param_spec = (param_id < param_size) ?
          &layer_param.param(param_id) : &default_param_spec;
      const bool param_need_backward = param_spec->lr_mult() != 0;
      need_backward |= param_need_backward;
      layers_[layer_id]->set_param_propagate_down(param_id,
                                                  param_need_backward);
    }
```
接着设置了每一层的 <code>propagate_down</code>
```
    for (int param_id = 0; param_id < num_param_blobs; ++param_id) {
      AppendParam(param, layer_id, param_id);
    }
```
再把每一层的参数添加进去。
```
    // Finally, set the backward flag
    layer_need_backward_.push_back(need_backward);
    if (need_backward) {
      for (int top_id = 0; top_id < top_id_vecs_[layer_id].size(); ++top_id) {
        blob_need_backward_[top_id_vecs_[layer_id][top_id]] = true;
      }
    }
```
最后设置一下反向传播的状态，如果参数里指定了这一层有反向传播那么这里就会设置成 <code>true</code>
这样我们这个对于layer初始化的过程就完成了
```
  // Go through the net backwards to determine which blobs contribute to the
  // loss.  We can skip backward computation for blobs that don't contribute
  // to the loss.
  // Also checks if all bottom blobs don't need backward computation (possible
  // because the skip_propagate_down param) and so we can skip backward
  // computation for the entire layer
```
然后他说通过网络反向传播去决定哪一个blob贡献了loss，就可以跳过这些不需要贡献loss的反传计算。
然后看看是不是所有的blob都不需要反传，如果是的话那么我们就可以直接跳过 <code>propagate_down</code>这一步了
```
    bool layer_contributes_loss = false;
    bool layer_skip_propagate_down = true;
```
首先默认他没有贡献loss的。
```
      if (layers_[layer_id]->loss(top_id) ||
          (blobs_under_loss.find(blob_name) != blobs_under_loss.end())) {
        layer_contributes_loss = true;
      }
```
如果发现它贡献了loss就把这个 <code>layer_contributes_loss</code>设置成 <code>true</code>
```
      if (blobs_skip_backp.find(blob_name) == blobs_skip_backp.end()) {
        layer_skip_propagate_down = false;
      }
      if (layer_contributes_loss && !layer_skip_propagate_down)
        break;
```
如果一直没有做贡献的话那么我们就可以跳过 <code>propagate_down</code>这一步了。
```
  // Handle force_backward if needed.
  if (param.force_backward()) {
    for (int layer_id = 0; layer_id < layers_.size(); ++layer_id) {
      layer_need_backward_[layer_id] = true;
      for (int bottom_id = 0;
           bottom_id < bottom_need_backward_[layer_id].size(); ++bottom_id) {
        bottom_need_backward_[layer_id][bottom_id] =
            bottom_need_backward_[layer_id][bottom_id] ||
            layers_[layer_id]->AllowForceBackward(bottom_id);
        blob_need_backward_[bottom_id_vecs_[layer_id][bottom_id]] =
            blob_need_backward_[bottom_id_vecs_[layer_id][bottom_id]] ||
            bottom_need_backward_[layer_id][bottom_id];
      }
      for (int param_id = 0; param_id < layers_[layer_id]->blobs().size();
           ++param_id) {
        layers_[layer_id]->set_param_propagate_down(param_id, true);
      }
    }
  }
```
然后这里处理了一下强制反传，就是设置一下哪些blob需要强制反传，哪些不需要。
```
  // In the end, all remaining blobs are considered output blobs.
  for (set<string>::iterator it = available_blobs.begin();
      it != available_blobs.end(); ++it) {
    LOG_IF(INFO, Caffe::root_solver())
        << "This network produces output " << *it;
    net_output_blobs_.push_back(blobs_[blob_name_to_idx[*it]].get());
    net_output_blob_indices_.push_back(blob_name_to_idx[*it]);
  }
```
然后又弄了一个做 <code>net_output_blobs_</code>的vector，就相当于把这个网络串联起来，类似于一个单链表结构。
```
  ShareWeights();
  debug_info_ = param.debug_info();
  LOG_IF(INFO, Caffe::root_solver()) << "Network initialization done.";
```
接着从这个参数里面抄权值，然后设置一下debug的状态。
最后这个 <code>Init</code>函数就结束了。
这样我们构造函数和 <code>Init</code>函数就结束了。

# FilterNet 函数
```
  // Helpers for Init.
  /**
   * @brief Remove layers that the user specified should be excluded given the current
   *        phase, level, and stage.
   */
  static void FilterNet(const NetParameter& param,
      NetParameter* param_filtered);
```
这个函数是根据 <code>state</code>初始化所有层的一个函数。<code>state</code>包含了 <code>phase, level stage</code>
```
  NetState net_state(param.state());
  param_filtered->CopyFrom(param);
```
这里拷贝了参数到 <code>param_filtered</code>这个变量里面
```
param_filtered->clear_layer();
```
然后调用 <code>clear_layer</code>清除所有层，因为它是刚刚开始初始化时候调用的，就需要把内存里的一些垃圾信息删掉。
```
for (int i = 0; i < param.layer_size(); ++i)
```
这里循环遍历所有层，然后
加载每一层的参数和名字
```
    const LayerParameter& layer_param = param.layer(i);
    const string& layer_name = layer_param.name();
```
检查每一层的 <code>StateMeetsRule</code>
```
    bool layer_included = (layer_param.include_size() == 0);
    for (int j = 0; layer_included && j < layer_param.exclude_size(); ++j) {
      if (StateMeetsRule(net_state, layer_param.exclude(j), layer_name)) {
        layer_included = false;
      }
    }
```
这个<code>StateMeetsRule</code>属于级联网络才用得到的开关，我们先跳过这一步。
然后开始copy layer
```
    if (layer_included) {
      param_filtered->add_layer()->CopyFrom(layer_param);
    }
```
之前不是把layer都clear了吗，现在就把从param里读的参数添加给layer。

# Forward 函数
```
  /**
   * @brief Run Forward and return the result.
   *
   */
  const vector<Blob<Dtype>*>& Forward(Dtype* loss = NULL);
```
这个是 <code>Net</code>的 <code>Forward</code>函数，不是 <code>Layer</code>的<code>Forward</code>函数，<code>Net</code>的 <code>Forward</code>函数是指挥每一个 <code>Layer</code>往哪个地方去传。
```
template <typename Dtype>
const vector<Blob<Dtype>*>& Net<Dtype>::Forward(Dtype* loss) {
  if (loss != NULL) {
    *loss = ForwardFromTo(0, layers_.size() - 1);
  } else {
    ForwardFromTo(0, layers_.size() - 1);
  }
  return net_output_blobs_;
}
```
这里判断了一下需不需要 <code>loss</code>,如果不需要 <code>loss</code>就
直接走这个 <code>ForwardFromTo</code>这个函数
# ForwardFromTo 函数
我们可以进来看一下
```
for (int c = 0; c < before_forward_.size(); ++c) {
      before_forward_[c]->run(i);
    }
    Dtype layer_loss = layers
```
首先让 <code>before_forward_</code>的这些步骤先走，因为他们的名字就是在前传之前的步骤，他们走完之后就开始走 <code>layer</code>的步骤
```
    Dtype layer_loss = layers_[i]->Forward(bottom_vecs_[i], top_vecs_[i]);
    loss += layer_loss;
```
然后 <code>layer</code>挨个走完 <code>Forward</code>之后，有个 <code>loss</code>把每一步额的 <code>loss</code>加起来。
这个走完之后就是 <code>after_forward</code>
```
    for (int c = 0; c < after_forward_.size(); ++c) {
      after_forward_[c]->run(i);
    }
```
也是顾名思义，所以他们在 <code>forward</code>之后也走了一遍。
最后把总的 <code>loss</code>返回就可以了。
```
return loss;
```
因为这个是net的 <code>Forward</code>函数，他就是一个大的框架，起的是串联的作用，所以比较简单。

# ForwardPrefilled 函数
他的注释上写了,使用 <code>Forward</code>代替
```
  /// @brief DEPRECATED; use Forward() instead.
  const vector<Blob<Dtype>*>& ForwardPrefilled(Dtype* loss = NULL) {
    LOG_EVERY_N(WARNING, 1000) << "DEPRECATED: ForwardPrefilled() "
        << "will be removed in a future version. Use Forward().";
    return Forward(loss);
  }
```
而且他的实现也就是直接调用的 <code>Forward</code>函数，所以这个函数实际上是一个比较老的函数，他只是留下了一个接口而已，以后都不会再用到了。

# ForwardFromTo 函数
```
  /**
   * The From and To variants of Forward and Backward operate on the
   * (topological) ordering by which the net is specified. For general DAG
   * networks, note that (1) computing from one layer to another might entail
   * extra computation on unrelated branches, and (2) computation starting in
   * the middle may be incorrect if all of the layers of a fan-in are not
   * included.
   */
  Dtype ForwardFromTo(int start, int end);
```
这个函数注释上就写的很明白了，就是指定从某一层到某一层的一个函数，与之类似的还有 <code>ForwardFrom</code> <code>ForwardTo</code>。都只是参数不同而已。

# Forward 函数
这里是另外的一个 <code>Forward</code>函数
```
  /// @brief DEPRECATED; set input blobs then use Forward() instead.
  const vector<Blob<Dtype>*>& Forward(const vector<Blob<Dtype>* > & bottom,
      Dtype* loss = NULL);
```
可以看到，他把 <code>loss</code>放在了参数里面,这个也是一个比较老的实现方法了，现在也都是用之前哪个 <code>Forward</code>函数来实现了。
```
template <typename Dtype>
const vector<Blob<Dtype>*>& Net<Dtype>::Forward(
    const vector<Blob<Dtype>*> & bottom, Dtype* loss) {
  LOG_EVERY_N(WARNING, 1000) << "DEPRECATED: Forward(bottom, loss) "
      << "will be removed in a future version. Use Forward(loss).";
  // Copy bottom to net bottoms
  for (int i = 0; i < bottom.size(); ++i) {
    net_input_blobs_[i]->CopyFrom(*bottom[i]);
  }
  return Forward(loss);
}
```
以前是会把 <code>bottom</code>都传进来的，而现在 <code>bottom</code>是在初始化的时候就加载了，所以不用传了。但是保留了原来的接口，所以主页里还是先复制一下它传进来的 <code>bottom</code>，然后再 <code>Forward</code>

# ClearParamDiffs 函数
```
  /**
   * @brief Zeroes out the diffs of all net parameters.
   *        Should be run before Backward.
   */
  void ClearParamDiffs();
```
他说了，这里是把所有的参数都清成0，这个必须得在反传之前运行。

# Backward 函数
反传函数其实和前传是一样的，因为这个是net的反传，它只是一个架构的作用
```
BackwardFromTo(layers_.size() - 1, 0);
```
可以看到，它第一步就是调用这个 <code>BackwardFromTo</code>函数，实际上和前传是一样的。
```
  if (debug_info_) {
    Dtype asum_data = 0, asum_diff = 0, sumsq_data = 0, sumsq_diff = 0;
    for (int i = 0; i < learnable_params_.size(); ++i) {
      asum_data += learnable_params_[i]->asum_data();
      asum_diff += learnable_params_[i]->asum_diff();
      sumsq_data += learnable_params_[i]->sumsq_data();
      sumsq_diff += learnable_params_[i]->sumsq_diff();
    }
    const Dtype l2norm_data = std::sqrt(sumsq_data);
    const Dtype l2norm_diff = std::sqrt(sumsq_diff);
    LOG(ERROR) << "    [Backward] All net params (data, diff): "
               << "L1 norm = (" << asum_data << ", " << asum_diff << "); "
               << "L2 norm = (" << l2norm_data << ", " << l2norm_diff << ")";
  }
}
```
然后下面这些都是需要debug的一些参数，比方说各个data，各个diff，还有第二范式等等。

# BackwardFromTo 函数
```
template <typename Dtype>
void Net<Dtype>::BackwardFromTo(int start, int end) {
  CHECK_GE(end, 0);
  CHECK_LT(start, layers_.size());
  for (int i = start; i >= end; --i) {
    for (int c = 0; c < before_backward_.size(); ++c) {
      before_backward_[c]->run(i);
    }
    if (layer_need_backward_[i]) {
      layers_[i]->Backward(
          top_vecs_[i], bottom_need_backward_[i], bottom_vecs_[i]);
      if (debug_info_) { BackwardDebugInfo(i); }
    }
    for (int c = 0; c < after_backward_.size(); ++c) {
      after_backward_[c]->run(i);
    }
  }
}
```
这个 <code>BackwardFromTo</code>函数和前传是一样的，设置一个起点，设置一个终点，然后运行 <code>before_backward_</code>，完了运行 <code>Backward</code>，最后运行 <code>after_backward_</code>

# Reshape 函数
```
  /**
   * @brief Reshape all layers from bottom to top.
   *
   * This is useful to propagate changes to layer sizes without running
   * a forward pass, e.g. to compute output feature size.
   */
  void Reshape();
```
这里是net的Reshape，不是blob的Reshape。这里说Reshape会改变所有的layer的 top bottom的shape，也就是说你你前传的时候如果就没有改变这个layer的大小的话就会调用这个函数。
```
template <typename Dtype>
void Net<Dtype>::Reshape() {
  for (int i = 0; i < layers_.size(); ++i) {
    layers_[i]->Reshape(bottom_vecs_[i], top_vecs_[i]);
  }
}
```
可以看到，它的实现非常简单，就是遍历了所有layer，每个layer都Reshape了一下。
因为layer的Reshape是调整了一下top的形状，然后给他分配缓存空间。

# ForwardBackward 函数
这个函数就是 <code>Forward</code>和 <code>Backward</code>结合起来
```
  Dtype ForwardBackward() {
    Dtype loss;
    Forward(&loss);
    Backward();
    return loss;
  }
```
最后他也会返回一个前传得到的loss，这一步是实现好了的，可以方便的在训练时候去调它。

# Update 函数
```
  /// @brief Updates the network weights based on the diff values computed.
  void Update();
```
这个 <code>Updata</code>函数是根据计算的diff来更新网络的权重
```
template <typename Dtype>
void Net<Dtype>::Update() {
  for (int i = 0; i < learnable_params_.size(); ++i) {
    learnable_params_[i]->Update();
  }
}
```
它调用的是 <code>learnable_params_</code>的 <code>Update</code>,而这个<code>learnable_params_</code>的 <code>Update</code>是 <code>Blob</code>里的函数，他就是计算权重的一个函数。

# ShareWeights 函数
```
  /**
   * @brief Shares weight data of owner blobs with shared blobs.
   *
   * Note: this is called by Net::Init, and thus should normally not be
   * called manually.
   */
  void ShareWeights();
```
这个函数是从参数里面读取那些权重，这个函数只在 <code>Net::Init</code>里调一次，所以不应该手动去调用它。

# ShareTrainedLayersWith 函数
```
  /**
   * @brief For an already initialized net, implicitly copies (i.e., using no
   *        additional memory) the pre-trained layers from another Net.
   */
  void ShareTrainedLayersWith(const Net* other);
```
它的注释是说从已经初始化了的网络，隐式的去拷贝，不占用内存，也就内存映射的意思，其实就是加载一个pretrain的模型。
```
Layer<Dtype>* source_layer = other->layers()[i].get();
```
可以看到，它的指针指向了传进来的那个layer，这个意思就是内存映射的意思。

# copy 函数
```
// For an already initialized net, CopyTrainedLayersFrom() copies the already
  // trained layers from another net parameter instance.
  /**
   * @brief For an already initialized net, copies the pre-trained layers from
   *        another Net.
   */
  void CopyTrainedLayersFrom(const NetParameter& param);
  void CopyTrainedLayersFrom(const string& trained_filename);
  void CopyTrainedLayersFromBinaryProto(const string& trained_filename);
  void CopyTrainedLayersFromHDF5(const string& trained_filename);
```
这些都是拷贝的函数，就是从文件里面拷贝一个网络到对象里面。

# toProto 函数
```
  /// @brief Writes the net to a proto.
  void ToProto(NetParameter* param, bool write_diff = false) const;
  /// @brief Writes the net to an HDF5 file.
  void ToHDF5(const string& filename, bool write_diff = false) const;
```
这些是把网络储存到文件中的两个函数，一个是proto一个是HDF5格式

剩下的函数都是返回成员变量的一些函数了，比方说返回名字等等。