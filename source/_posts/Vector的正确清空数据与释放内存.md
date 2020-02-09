---
title: Vector的正确清空数据与释放内存
date: 2020-02-08 15:17:43
tags:
---
转载自：[嚜寒的CSDN](https://blog.csdn.net/a272846945/article/details/51182144)

0）简单介绍

在vector的数据结构中，  

.clear();清空数据

.size();当前vector容器内存储的元素的个数

.capacity();当前vector容器重新分配内存之前所能容纳的元素数量  

.swap();函数交换

  

1）问题

在用vector做题时，输入完一组数据处理完后，及时clear()，然后输入下一组数据，但是如果在输入之前，输出之前vector所存的内容，会发现仍然存在，但是如果输出.empty();它会返回1，告诉我们这个容器现在是空的，这是因为使用.clear()清空内容，但是没有释放内存的原因。举例如下：

[cpp] [view plain](https://blog.csdn.net/a272846945/article/details/51182144#
"view plain")
[copy](https://blog.csdn.net/a272846945/article/details/51182144# "copy")

  1. #include <iostream>
  2. #include <vector>
  3.   4. using namespace std; 
  5. int main() 
  6. { 
  7. vector <int >a; 
  8. cout<<a.empty()<<endl;//输出 1 代表该vector此时是空
  9. a.push_back(1); 
  10. a.push_back(2); 
  11. cout<<a[0]<<" "<<a[1]<<endl;//输出1 2
  12. cout<<a.empty()<<endl;//输出 0 代表该vector此时非空
  13. cout<<a.size()<<endl;//输出2
  14. cout<<a.capacity()<<endl;//输出2
  15. cout<<"***************"<<endl; 
  16.   17. //a[0]=NULL;a[1]=NULL; 这是赋值为0，并不清空数据，也不释放内存。
  18. a.clear(); 
  19. cout<<a[0]<<" "<<a[1]<<endl;//仍然输出1 2，因为没有释放内存，所以输出该地址的内容仍然与之前一样
  20. cout<<a.empty()<<endl;//输出1 代表该vector此时已经为空
  21. cout<<a.size()<<endl;//输出0，代表当前容器内存储元素个数是0，与.empty()类似，都告诉我们当前容器是空的意思
  22. cout<<a.capacity()<<endl;//输出2，代表当前该vector在重新分配存储空间前所能容纳的元素数量并没有改变
  23. cout<<"***************"<<endl; 
  24.   25. /*
  26. 下面这五行说明，.pop_back()与.clear()起到了相同的作用，都是清空数据，但是没有释放内存
  27. while(!a.empty()){
  28. a.pop_back();
  29. }
  30. cout<<a.empty()<<endl;//输出 1 代表该vector此时已经为空
  31. cout<<a[0]<<" "<<a[1]<<endl;//仍然输出为 1 2，因为没有释放内存，所以输出该地址的内容仍然与之前一样
  32. */
  33. a.push_back(4); 
  34. cout<<a[0]<<" "<<a[1]<<" "<<a[2]<<endl;//输出 4 2 0 尽管没有释放内存，但是已经认为该vector已经被清空，所以再push_back();时，a[0]被覆盖。
  35. cout<<a.size()<<endl;//输出1，代表当前容器内存储元素个数是1，就是刚刚push_back();装进去的数起到的作用
  36. cout<<a.capacity()<<endl;//此时仍然输出2
  37. cout<<"***************"<<endl; 
  38.   39. //那么如何释放内存呢？我们用swap交换到一个新的类型的vector,将原来的a拷贝出去，然后自然销毁，而新的到的a是全新的没有存任何数据的
  40. vector<int>().swap(a); 
  41. //a.swap();
  42. cout<<a.size()<<endl;//输出 0
  43. cout<<a.capacity()<<endl;.// 输出 0
  44. //cout<<a[1]<<endl;
  45.   46. } 

  

