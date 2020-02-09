头文件

    
    
    #include <thread>

示例代码：

Search_param search_param[6];

std::vector<std::thread> threads;

std::vector<return_param> ret_param;

for(int i = 0; i < 6; i++)

{

search_param[i] = {1, "abc", ret_param[i]};

std::thread find_border_thread(find_border, std::ref(search_param[i]));

threads.emplace_back(std::move(find_border_thread));

}

for (int i = 0; i < 6; i++)

{

threads[i].join();

}

  

void find_border(Search_param &param)

{

printf("I'm in find_border");

}

使用join可以等待线程结束才执行下一步的代码，线程执行是从声明thread这个对象时候就开始了。

把要传的参数卸载结构体对象里一起传进去，要得到的返回值也写在结构体里用引用传进去就可以了。

