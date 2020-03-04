---
title: caffe源码阅读《五》solver
date: 2020-02-29 10:09:02
tags: [caffe]
categories: 
- caffe源码解读
---
# SolverAction
最开始定义了一个枚举
```
/**
  * @brief Enumeration of actions that a client of the Solver may request by
  * implementing the Solver's action request function, which a
  * client may optionally provide in order to request early termination
  * or saving a snapshot without exiting. In the executable caffe, this
  * mechanism is used to allow the snapshot to be saved when stopping
  * execution with a SIGINT (Ctrl-C).
  */
  namespace SolverAction {
    enum Enum {
      NONE = 0,  // Take no special action.
      STOP = 1,  // Stop training. snapshot_after_train controls whether a
                 // snapshot is created.
      SNAPSHOT = 2  // Take a snapshot, and keep training.
    };
  }
```
枚举了一些动作，有时候我们的客户端可能会提前终止程序，例如我们使用 <code>ctrl+c</code>把程序终止了，他就需要做出一些响应，例如生成快照，快速保存等等。那么就需要使用这几个状态来判断
* None就是没有什么异常
* STOP就是训练停止了
* SNAPSHOT就是创建一个SNAPSHOT然后我们接着去训练

# ActionCallback 类型
这里定义了一个新的类型，给需要用到那几个状态的回调函数使用
```
/**
 * @brief Type of a function that returns a Solver Action enumeration.
 */
typedef boost::function<SolverAction::Enum()> ActionCallback;
```
用来在出了任何异常的时候调用相应状态的应急措施，看它的参数就是上面定义的那个枚举的状态码。

# Solver类
```
/**
 * @brief An interface for classes that perform optimization on Net%s.
 *
 * Requires implementation of ApplyUpdate to compute a parameter update
 * given the current state of the Net parameters.
 */
template <typename Dtype>
class Solver
```
注释上写了这个类是个优化网络的一个借口，需要自己去实现 <code>ApplyUpdate</code>这个函数来实现具体的参数更新的过程。

## Solver 构造函数
```
  explicit Solver(const SolverParameter& param);
  explicit Solver(const string& param_file);
```
他也是两个构造函数，一个是从参数读取，一个是从文件读取。
```
template <typename Dtype>
Solver<Dtype>::Solver(const string& param_file)
    : net_(), callbacks_(), requested_early_exit_(false) {
  SolverParameter param;
  ReadSolverParamsFromTextFileOrDie(param_file, &param);
  Init(param);
}
```
可以看到，这个函数就是从文件里去读，然后放在 <code>SolverParameter</code> 类型的变量里，最后调用了 <code>Init</code>这个函数。
```
template <typename Dtype>
Solver<Dtype>::Solver(const SolverParameter& param)
    : net_(), callbacks_(), requested_early_exit_(false) {
  Init(param);
}
```
如果直接传的就是这个类型的对象的话那么他就直接调用 <code>Init</code>这个函数了，所以其实这两个函数实现的功能是一样的。
接下来我们重点来看这个 <code>Init</code>函数。

## Init 函数
```
void Init(const SolverParameter& param);
```
这个函数根据读取的参数来初始化网络
```
  CHECK_GE(param_.average_loss(), 1) << "average_loss should be non-negative.";
  CheckSnapshotWritePermissions();
  if (param_.random_seed() >= 0) {
    Caffe::set_random_seed(param_.random_seed() + Caffe::solver_rank());
  }
```
首先做了一些检查
* 平均loss不能是负数
* 快照保存路径是否有写入的权限
* 随机种子是否设置了
```
  // Scaffolding code
  InitTrainNet();
  InitTestNets();
```
接着初始化了训练和测试的网络，这两个函数在之后也详细解析了。

## CheckSnapshotWritePermissions 函数
```
template <typename Dtype>
void Solver<Dtype>::CheckSnapshotWritePermissions() {
  if (Caffe::root_solver() && param_.snapshot()) {
    CHECK(param_.has_snapshot_prefix())
        << "In solver params, snapshot is specified but snapshot_prefix is not";
    string probe_filename = SnapshotFilename(".tempfile");
    std::ofstream probe_ofs(probe_filename.c_str());
    if (probe_ofs.good()) {
      probe_ofs.close();
      std::remove(probe_filename.c_str());
    } else {
      LOG(FATAL) << "Cannot write to snapshot prefix '"
          << param_.snapshot_prefix() << "'.  Make sure "
          << "that the directory exists and is writable.";
    }
  }
}
```
这个函数是生成了一个 <code>tempfile</code>把它写入这个地方看看，如果文件顺利生成的话就把他给删掉，如果有错误的话就抛出异常，提示不能写他，看看是不是没有权限写。

## InitTrainNet 函数
首先检查了一下有没有训练网络，如果没有训练网络他会直接报错的。
```
  const int num_train_nets = param_.has_net() + param_.has_net_param() +
      param_.has_train_net() + param_.has_train_net_param();
  const string field_names = "net, net_param, train_net, train_net_param";
  CHECK_GE(num_train_nets, 1) << "SolverParameter must specify a train net "
      << "using one of these fields: " << field_names;
  CHECK_LE(num_train_nets, 1) << "SolverParameter must not contain more than "
      << "one of these fields specifying a train_net: " << field_names;
```
然后拷贝网络的参数,如果有网络结构就直接拷贝网络结构。
```
  NetParameter net_param;
  if (param_.has_train_net_param()) {
    LOG_IF(INFO, Caffe::root_solver())
        << "Creating training net specified in train_net_param.";
    net_param.CopyFrom(param_.train_net_param());
  } else if (param_.has_train_net()) {
    LOG_IF(INFO, Caffe::root_solver())
        << "Creating training net from train_net file: " << param_.train_net();
    ReadNetParamsFromTextFileOrDie(param_.train_net(), &net_param);
  }
  if (param_.has_net_param()) {
    LOG_IF(INFO, Caffe::root_solver())
        << "Creating training net specified in net_param.";
    net_param.CopyFrom(param_.net_param());
  }
  if (param_.has_net()) {
    LOG_IF(INFO, Caffe::root_solver())
        << "Creating training net from net file: " << param_.net();
    ReadNetParamsFromTextFileOrDie(param_.net(), &net_param);
  }
```
因为caffe里面网络有的是指定了专门训练用的，有的是指定了专门测试用的，有的没有指定就是通用的，通用的就是训练和测试都可以使用。
然后拷贝网络的状态到 <code>NetState</code>里面
```
  // Set the correct NetState.  We start with the solver defaults (lowest
  // precedence); then, merge in any NetState specified by the net_param itself;
  // finally, merge in any NetState specified by the train_state (highest
  // precedence).
  NetState net_state;
  net_state.set_phase(TRAIN);
  net_state.MergeFrom(net_param.state());
  net_state.MergeFrom(param_.train_state());
  net_param.mutable_state()->CopyFrom(net_state);
```
最后用这些初始化的参数把整个网络重新设置一遍
```
  net_.reset(new Net<Dtype>(net_param));
```
如果传入了权重的话还会去读取权重
```
  for (int w_idx = 0; w_idx < param_.weights_size(); ++w_idx) {
    LoadNetWeights(net_, param_.weights(w_idx));
  }
```

## LoadNetWeights 函数
这个函数会读取权重信息
```
  std::vector<std::string> model_names;
  boost::split(model_names, model_list, boost::is_any_of(","));
  for (int i = 0; i < model_names.size(); ++i) {
    boost::trim(model_names[i]);
    LOG(INFO) << "Finetuning from " << model_names[i];
    net->CopyTrainedLayersFrom(model_names[i]);
  }
```
如果你的model是多个文件的话，他还会根据逗号去分割，然后从多个model中读取权重信息。

## InitTestNets 函数
这个函数和 <code>InitTrainNets</code>很像，判断了一下哪些层是test的或者是通用的，就把它加载进来。

## SetActionFunction 函数
这个函数就是交给用户去实现的，就是在有信号输入的时候该做什么处理，由用户来决定。
```
template<typename Dtype>
void Solver<Dtype>::SetActionFunction(ActionCallback func) {
  action_request_function_ = func;
}
```
它的实现就是把这个函数传给我们这个 <code>action_request_function_</code>，这就是把用户写的回调函数的指针传进来，当遇到状态的时候就调用这个函数。

## GetRequestedAction 函数
这个函数是有返回值的
```
  SolverAction::Enum GetRequestedAction();
```
如果你定义了自己的状态处理函数，那么就执行你写的函数，然后返回你给的返回值，如果没有定义，就返回 <code>SolverAction::NONE</code>
```
template<typename Dtype>
SolverAction::Enum Solver<Dtype>::GetRequestedAction() {
  if (action_request_function_) {
    // If the external request function has been set, call it.
    return action_request_function_();
  }
  return SolverAction::NONE;
}
```

## Solve 函数
```
  // The main entry of the solver function. In default, iter will be zero. Pass
  // in a non-zero iter number to resume training for a pre-trained net.
  virtual void Solve(const char* resume_file = NULL);
  inline void Solve(const string& resume_file) { Solve(resume_file.c_str()); }
```
这个函数其实就是 <code>Solver</code>这个类的入口函数，我们之前在讲caffe训练程序入口的时候也说了，实际上核心就是这个 <code>Solve</code>函数。它的唯一一个参数就是一个文件名，就是如果之前有保存的配置文件就传入读取，如果没有的话就设置为 <code>NULL</code>.
```
  CHECK(Caffe::root_solver());
  LOG(INFO) << "Solving " << net_->name();
  LOG(INFO) << "Learning Rate Policy: " << param_.lr_policy();

  // Initialize to false every time we start solving.
  requested_early_exit_ = false;

  if (resume_file) {
    LOG(INFO) << "Restoring previous solver status from " << resume_file;
    Restore(resume_file);
  }
```
代码里很明显，一开始就是判断是否传入了文件路径，如果有的话就会读取。
如果我们是读取的存档，那么起点的计数器就是有值的了
```
  int start_iter = iter_;
```
然后调用 <code>Step</code>函数
```
Step(param_.max_iter() - iter_);
```
这个<code>Step</code>函数就是主循环所在的位置
全部训练完了以后就开始存快照
```
  // If we haven't already, save a snapshot after optimization, unless
  // overridden by setting snapshot_after_train := false
  if (param_.snapshot_after_train()
      && (!param_.snapshot() || iter_ % param_.snapshot() != 0)) {
    Snapshot();
  }
```
然后该打印的就打印了
```
  // After the optimization is done, run an additional train and test pass to
  // display the train and test loss/outputs if appropriate (based on the
  // display and test_interval settings, respectively).  Unlike in the rest of
  // training, for the train net we only run a forward pass as we've already
  // updated the parameters "max_iter" times -- this final pass is only done to
  // display the loss, which is computed in the forward pass.
  if (param_.display() && iter_ % param_.display() == 0) {
    int average_loss = this->param_.average_loss();
    Dtype loss;
    net_->Forward(&loss);

    UpdateSmoothedLoss(loss, start_iter, average_loss);

    LOG(INFO) << "Iteration " << iter_ << ", loss = " << smoothed_loss_;
  }
```
最后该测试的就测试
```
  if (param_.test_interval() && iter_ % param_.test_interval() == 0) {
    TestAll();
  }
  LOG(INFO) << "Optimization Done.";
```
这样整个训练的流程就结束了。

## Step 函数
这个函数就是训练的主循环了，参数就是还需要执行多少步停止
```
template <typename Dtype>
void Solver<Dtype>::Step(int iters) {
  const int start_iter = iter_;
  const int stop_iter = iter_ + iters;
```
在这个函数里有一个主循环
```
while (iter_ < stop_iter)
```
如果当前的次数正好符合我们测试的规则的话就会去测一波。
```
    if (param_.test_interval() && iter_ % param_.test_interval() == 0
        && (iter_ > 0 || param_.test_initialization())) {
      if (Caffe::root_solver()) {
        TestAll();
      }
      if (requested_early_exit_) {
        // Break out of the while loop because stop was requested while testing.
        break;
      }
    }
```
当然，如果提前终止了就跳出这个程序，进入提前终止的事件。
```
    for (int i = 0; i < callbacks_.size(); ++i) {
      callbacks_[i]->on_start();
    }
```
这里有个 <code>callbacks_</code>,也是在这个文件里，定义了一个 <code>Callback</code>的类，就是方便用户来干一些事情放在这个 <code>Step</code>中间的，当然，如果用户不需要的话不实现就会跳过。
接着判断一下到没到打印的次数
```
    const bool display = param_.display() && iter_ % param_.display() == 0;
    net_->set_debug_info(display && param_.debug_info());
```
再接着就开始前传和回传了
```
    // accumulate the loss and gradient
    Dtype loss = 0;
    for (int i = 0; i < param_.iter_size(); ++i) {
      loss += net_->ForwardBackward();
    }
```
可以看到这里有个循环把loss叠加起来，迭代了 <code>iter_size</code>次，这个 <code>iter_size</code>是为了满足大的batch_size却没有足够大的显存的需求。比方说我们想要设置一个128的batch,但是我们显存之后一次放2张图，那么就可以设置batch_size为2，然后设置iter_size为64来代替直接设置成128。所以我们计算loss的时候是把他全部加起来然后平均一下。
```
    loss /= param_.iter_size();
    // average the loss across iterations for smoothed reporting
    UpdateSmoothedLoss(loss, start_iter, average_loss);
```
为什么我们走了 <code>UpdateSmoothedLoss</code>呢，它是对全样本的loss再更新一次，不仅仅限制于当前这次迭代，它是把之前计算的loss也加进去平均了，防止某一次突然出现较大的变化把loss带偏了。
之后就如果满足打印的次数，就开始打印了。
打印完之后也给用户留了一个自己实现函数的机会
```
    for (int i = 0; i < callbacks_.size(); ++i) {
      callbacks_[i]->on_gradients_ready();
    }
```
最后就可以更新我们的权重了
```
    ApplyUpdate();
```
但是这个函数我们在 <code>Solver</code>这个类里面是没有实现的，他也是在子类中具体实现，根据不同的学习率算法，例如SGD就在 <code>SGDSolver</code>这个子类里面去具体实现
最后判断一下是不是要保存，然后这个函数就算结束了
```
    SolverAction::Enum request = GetRequestedAction();

    // Save a snapshot if needed.
    if ((param_.snapshot()
         && iter_ % param_.snapshot() == 0
         && Caffe::root_solver()) ||
         (request == SolverAction::SNAPSHOT)) {
      Snapshot();
    }
    if (SolverAction::STOP == request) {
      requested_early_exit_ = true;
      // Break out of training loop.
      break;
    }
```

## UpdateSmoothedLoss 函数
```
template <typename Dtype>
void Solver<Dtype>::UpdateSmoothedLoss(Dtype loss, int start_iter,
    int average_loss)
```
它传入了平均的loss，还有其实的迭代id，这样就可以算出总的一个平均loss了
```
  if (losses_.size() < average_loss) {
    losses_.push_back(loss);
    int size = losses_.size();
    smoothed_loss_ = (smoothed_loss_ * (size - 1) + loss) / size;
  } else {
    int idx = (iter_ - start_iter) % average_loss;
    smoothed_loss_ += (loss - losses_[idx]) / average_loss;
    losses_[idx] = loss;
  }
```
我们可以看到，它是把之前的loss也加进来了，然后一起又平均了一个 <code>smoothed_loss_</code>出来返回的。

# Callback 类
```
  // Invoked at specific points during an iteration
  class Callback {
   protected:
    virtual void on_start() = 0;
    virtual void on_gradients_ready() = 0;

    template <typename T>
    friend class Solver;
  };
```
这个类给我们用户提供了一个回调的机会，我们用户可以自己实现一段函数用来在训练的 <code>Step</code>中使用

