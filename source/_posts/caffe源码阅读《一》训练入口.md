---
title: caffe源码阅读《一》训练入口
date: 2020-02-08 15:10:55
top: 1
tags: [caffe]
categories: 
- caffe源码解读
---
随着工作的深入，很多时候调参不仅仅局限于乱改学习率上面了，阅读caffe源码变得至关重要，而我又比较讨厌盯着虚无的代码看，总想跑个程序跟着debug走，所以就选择了训练程序作为入口。

caffe的训练程序在tools目录里，名字就叫做caffe.cpp

训练的时候需要几个参数，最重要的有两个，一个是train还是test，另一个就是solver.prototxt文件了。

只到这个后，我们跟着源码走
```
    
    
    int main(int argc, char** argv) {
      // Print output to stderr (while still logging).
      FLAGS_alsologtostderr = 1;
      // Set version
      gflags::SetVersionString(AS_STRING(CAFFE_VERSION));
      // Usage message.
      gflags::SetUsageMessage("command line brew\n"
          "usage: caffe <command> <args>\n\n"
          "commands:\n"
          "  train           train or finetune a model\n"
          "  test            score a model\n"
          "  device_query    show GPU diagnostic information\n"
          "  time            benchmark model execution time");
      // Run tool or show usage.
      caffe::GlobalInit(&argc, &argv);
      if (argc == 2) {
    #ifdef WITH_PYTHON_LAYER
        try {
    #endif
          return GetBrewFunction(caffe::string(argv[1]))();
    #ifdef WITH_PYTHON_LAYER
        } catch (bp::error_already_set) {
          PyErr_Print();
          return 1;
        }
    #endif
      } else {
        gflags::ShowUsageWithFlagsRestrict(argv[0], "tools/caffe");
      }
    }
```
首先看主函数，主函数其实调用了GetBrewFunction这个函数，这个函数输入的是train，返回值实际上是一个函数指针，在上面有定义

```
    
    typedef int (*BrewFunction)();
    typedef std::map<caffe::string, BrewFunction> BrewMap;
    BrewMap g_brew_map;
    
    
    static BrewFunction GetBrewFunction(const caffe::string& name) {
      if (g_brew_map.count(name)) {
        return g_brew_map[name];
      } else {
        LOG(ERROR) << "Available caffe actions:";
        for (BrewMap::iterator it = g_brew_map.begin();
             it != g_brew_map.end(); ++it) {
          LOG(ERROR) << "\t" << it->first;
        }
        LOG(FATAL) << "Unknown action: " << name;
        return NULL;  // not reachable, just to suppress old compiler warnings.
      }
    }
```   
他的实际作用是返回<code>g_brew_map[name]</code>，如果传的参数是<code>train</code>那么这个<code>name</code>就是<code>train</code>，返回的呢就是<code>g_brew_map["train"]</code>
那么这个<code>g_brew_map</code>是什么呢？
看到定义
```
typedef std::map<caffe::string, BrewFunction> BrewMap;
```
它是一个map对象，它用来存储4种方法的函数指针。
在main函数之前，有一句
```
RegisterBrewFunction(device_query);
```
将四个函数进行了注册，注册的方法也在这个文件里面
```
#define RegisterBrewFunction(func) \
namespace { \
class __Registerer_##func { \
 public: /* NOLINT */ \
  __Registerer_##func() { \
    g_brew_map[#func] = &func; \
  } \
}; \
__Registerer_##func g_registerer_##func; \
}
```
所以我们在给GetBrewFunction传入的是train，那么就会返回train函数的指针，也就是执行train函数了。
我们先看这个<code>GetBrewFunction</code>函数，第一句
```
if (g_brew_map.count(name)) {
    return g_brew_map[name];
  }
```
这个<code>count</code>就是检查传入的<code>name</code>是否注册，如果已经注册了，那么就可以直接的返回这个函数指针了。
如果没有注册的话，那么就会打印所有的已注册函数名，并告诉你你传入的是什么，我们这里找不到。
```
else {
    LOG(ERROR) << "Available caffe actions:";
    for (BrewMap::iterator it = g_brew_map.begin();
         it != g_brew_map.end(); ++it) {
      LOG(ERROR) << "\t" << it->first;
    }
    LOG(FATAL) << "Unknown action: " << name;
    return NULL;  // not reachable, just to suppress old compiler warnings.
```
然后我们直接看<code>train</code>函数
``` 
    
    int train() {
      CHECK_GT(FLAGS_solver.size(), 0) << "Need a solver definition to train.";
      CHECK(!FLAGS_snapshot.size() || !FLAGS_weights.size())
          << "Give a snapshot to resume training or weights to finetune "
          "but not both.";
      vector<string> stages = get_stages_from_flags();
    
      caffe::SolverParameter solver_param;
      caffe::ReadSolverParamsFromTextFileOrDie(FLAGS_solver, &solver_param);
    
      solver_param.mutable_train_state()->set_level(FLAGS_level);
      for (int i = 0; i < stages.size(); i++) {
        solver_param.mutable_train_state()->add_stage(stages[i]);
      }
    
      // If the gpus flag is not provided, allow the mode and device to be set
      // in the solver prototxt.
      if (FLAGS_gpu.size() == 0
          && solver_param.has_solver_mode()
          && solver_param.solver_mode() == caffe::SolverParameter_SolverMode_GPU) {
          if (solver_param.has_device_id()) {
              FLAGS_gpu = "" +
                  boost::lexical_cast<string>(solver_param.device_id());
          } else {  // Set default GPU if unspecified
              FLAGS_gpu = "" + boost::lexical_cast<string>(0);
          }
      }
    
      vector<int> gpus;
      get_gpus(&gpus);
      if (gpus.size() == 0) {
        LOG(INFO) << "Use CPU.";
        Caffe::set_mode(Caffe::CPU);
      } else {
        ostringstream s;
        for (int i = 0; i < gpus.size(); ++i) {
          s << (i ? ", " : "") << gpus[i];
        }
        LOG(INFO) << "Using GPUs " << s.str();
    #ifndef CPU_ONLY
        cudaDeviceProp device_prop;
        for (int i = 0; i < gpus.size(); ++i) {
          cudaGetDeviceProperties(&device_prop, gpus[i]);
          LOG(INFO) << "GPU " << gpus[i] << ": " << device_prop.name;
        }
    #endif
        solver_param.set_device_id(gpus[0]);
        Caffe::SetDevice(gpus[0]);
        Caffe::set_mode(Caffe::GPU);
        Caffe::set_solver_count(gpus.size());
      }
    
      caffe::SignalHandler signal_handler(
            GetRequestedAction(FLAGS_sigint_effect),
            GetRequestedAction(FLAGS_sighup_effect));
    
      if (FLAGS_snapshot.size()) {
        solver_param.clear_weights();
      } else if (FLAGS_weights.size()) {
        solver_param.clear_weights();
        solver_param.add_weights(FLAGS_weights);
      }
    
      shared_ptr<caffe::Solver<float> >
          solver(caffe::SolverRegistry<float>::CreateSolver(solver_param));
    
      solver->SetActionFunction(signal_handler.GetActionFunction());
    
      if (FLAGS_snapshot.size()) {
        LOG(INFO) << "Resuming from " << FLAGS_snapshot;
        solver->Restore(FLAGS_snapshot.c_str());
      }
    
      LOG(INFO) << "Starting Optimization";
      if (gpus.size() > 1) {
    #ifdef USE_NCCL
        caffe::NCCL<float> nccl(solver);
        nccl.Run(gpus, FLAGS_snapshot.size() > 0 ? FLAGS_snapshot.c_str() : NULL);
    #else
        LOG(FATAL) << "Multi-GPU execution not available - rebuild with USE_NCCL";
    #endif
      } else {
        solver->Solve();
      }
      LOG(INFO) << "Optimization Done.";
      return 0;
    }
    RegisterBrewFunction(train);
```

首先是做检查，有没有参数文件，有没有从快照中读取权值等等。
```
CHECK_GT(FLAGS_solver.size(), 0) << "Need a solver definition to train.";
  CHECK(!FLAGS_snapshot.size() || !FLAGS_weights.size())
      << "Give a snapshot to resume training or weights to finetune "
      "but not both.";
```
这个solver文件就是我们传入的配置文件，通过gflag把我们的文件里的内容转化成FLAGS_solver变量。

然后执行
```    
    
      caffe::SolverParameter solver_param;
      caffe::ReadSolverParamsFromTextFileOrDie(FLAGS_solver, &solver_param);
```
把solver文件中的参数读取出来放在solver_param这个对象里。

然后就开始根据参数选择启用gpu还是cpu，加载网络，加载权值，加载步数等等。

直到这里

    
    
    shared_ptr<caffe::Solver<float> >
          solver(caffe::SolverRegistry<float>::CreateSolver(solver_param));

这里根据学习率下降策略返回一个函数指针，如果没有指定默认的就是sgd了，那么就可以去sgd这个文件里看一些代码，不过由于它是继承solver这个基类的，所以接下来要执行的还是在solver这个类里面找

他会执行step函数开始训练

    
    
    template <typename Dtype>
    void Solver<Dtype>::Step(int iters) {
      const int start_iter = iter_;
      const int stop_iter = iter_ + iters;
      int average_loss = this->param_.average_loss();
      losses_.clear();
      smoothed_loss_ = 0;
      iteration_timer_.Start();
      // 从开始到结束的大循环
      while (iter_ < stop_iter) {
        // zero-init the params
        net_->ClearParamDiffs();
        // 如果设置启动时先测试一波则开始第一次测试
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
    
        for (int i = 0; i < callbacks_.size(); ++i) {
          callbacks_[i]->on_start();
        }
        // 计算一下现在是不是该display了
        const bool display = param_.display() && iter_ % param_.display() == 0;
        net_->set_debug_info(display && param_.debug_info());
        // accumulate the loss and gradient
        // 算前传的loss
        Dtype loss = 0;
        for (int i = 0; i < param_.iter_size(); ++i) {
          loss += net_->ForwardBackward();
        }
        loss /= param_.iter_size();
        // average the loss across iterations for smoothed reporting
        // 更新loss
        UpdateSmoothedLoss(loss, start_iter, average_loss);
        if (display) {
          float lapse = iteration_timer_.Seconds();
          float per_s = (iter_ - iterations_last_) / (lapse ? lapse : 1);
          LOG_IF(INFO, Caffe::root_solver()) << "Iteration " << iter_
              << " (" << per_s << " iter/s, " << lapse << "s/"
              << param_.display() << " iters), loss = " << smoothed_loss_;
          iteration_timer_.Start();
          iterations_last_ = iter_;
          const vector<Blob<Dtype>*>& result = net_->output_blobs();
          int score_index = 0;
          for (int j = 0; j < result.size(); ++j) {
            const Dtype* result_vec = result[j]->cpu_data();
            const string& output_name =
                net_->blob_names()[net_->output_blob_indices()[j]];
            const Dtype loss_weight =
                net_->blob_loss_weights()[net_->output_blob_indices()[j]];
            for (int k = 0; k < result[j]->count(); ++k) {
              ostringstream loss_msg_stream;
              if (loss_weight) {
                loss_msg_stream << " (* " << loss_weight
                                << " = " << loss_weight * result_vec[k] << " loss)";
              }
              LOG_IF(INFO, Caffe::root_solver()) << "    Train net output #"
                  << score_index++ << ": " << output_name << " = "
                  << result_vec[k] << loss_msg_stream.str();
            }
          }
        }
        for (int i = 0; i < callbacks_.size(); ++i) {
          callbacks_[i]->on_gradients_ready();
        }
        ApplyUpdate();
    
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
      }
    }
    

这个函数就是整个主循环所在之处，也就是先初始化step，以及权值等信息，然后就开始学习，反向传播，更新权值等等。由于方法名字都能够顾名思义，所以不用语言赘述一遍了。函数虽然长，但是想要找什么顺着名字很容易都能找到。

我在学习的时候参照了

<https://www.cnblogs.com/liuzhongfeng/p/7289956.html>

博客，深受启发。

我在阅读的时候是录制了视频的，大家也可以去b站看到更详细的记录。
https://www.bilibili.com/video/av61042416/
<iframe src="//player.bilibili.com/player.html?aid=61042416&cid=106113982&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>

