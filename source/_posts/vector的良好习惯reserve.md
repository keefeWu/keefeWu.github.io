---
title: vector的良好习惯reserve
date: 2020-02-08 15:17:54
tags:
---
转载自：[点击打开链接](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870)

我先总结一下吧，就是如果要用vector不停的push_back，而且之前知道大概有多大，不一定要准确的，那么最好就先reserve一下，给它分配一定的空间，这样每次push_back不会真的分配内存，知道空间用完了才会自动分配内存。

总结一下，就是循环push_back前先reserve一个大概的大小，这个操作不会影响size，后面该push_back还是push_back。

具体原因看我转载的详细内容：

对于C++的vector容器模板类，存在size和capacity这样两个概念，可以分别通过vector的size()和capacity()方法获得该vector当前的size和capacity值。相应的，vector提供了两个方法来分别对size和capacity进行操作，它们就是resize方法和reserve方法。

  

首先，对于size和capacity，这是两个比较容易混淆的概念。都说要抱着问题来学习，才能做到事半功倍。那么，这里便提出三个问题：什么是vector的大小(即size)？什么是vector的容量(即capacity)？这两个概念的区别在哪里？

  

对于抽象的问题，只要我们把它们同我们的生活实际相结合，将问题具象化，自然就会很好的理解。

就拿我们的办公室举例，假设，我们部门的办公地点位于公司大楼的六楼。在我们的办公室里面，放置了100套办公桌椅(工位)，公司说按照一个萝卜一个坑来算，你们部门最多只能招这么多人，那么，这时我们可以说，我们部门的容量(即capacity)就是100人，如果我们部门是公司刚成立的部门，正处于发展壮大的阶段，目前只有40为员工，也就是说，办公室里只坐了40个人，另外60个工位是空着的，那么，我们可以说，我们部门当前的大小(即size)是40人。这实际上就是size和capacity的区别。类比到vector，size和capacity的概念自然就很清楚了。

  

cplusplus.com中对capacity是这样定义的：

This capacity is not necessarily equal to the [vector
size](http://www.cplusplus.com/vector::size). It can be equal or greater, with
the extra space allowing to accommodate for growth without the need to
reallocate on each insertion.  

一个allowing道出了真谛！这里还要区分两个概念，就是：为vector分配的存储空间和vector的大小是两个不同的概念。为vector分配的存储空间，实际上就是capacity，指的是当前vector最多能使用的存储空间，是大于等于vector的大小的，当vector实际需要使用的存储空间大于当前分配给它的存储空间时，需要重新为其分配存储空间。

  

cplusplus.com中对size的定义是：

This is the number of actual objects held in the
[vector](http://www.cplusplus.com/vector), which is not necessarily equal to
its storage [capacity](http://www.cplusplus.com/vector::capacity).  

实际上就是vector中当前实际存储的元素个数。

  

弄清楚了size和capacity这两个概念之后，对于resize和reserve两个方法就很好理解了。

cplusplus.com中对reserve的定义是：

Request a change in capacity

Requests that the [vector capacity](http://www.cplusplus.com/vector::capacity)
be at least enough to contain n elements.  
  
If n is greater than the current [vector
capacity](http://www.cplusplus.com/vector::capacity), the function causes the
container to reallocate its storage increasing
its[capacity](http://www.cplusplus.com/vector::capacity) to n (or greater).  
  
In all other cases, the function call does not cause a reallocation and the
[vector capacity](http://www.cplusplus.com/vector::capacity) is not affected.  
  
This function has no effect on the [vector
size](http://www.cplusplus.com/vector::size) and cannot alter its elements.

  

从上面的说明中，可以得到以下信息：

1、reserve方法被用来重新分配vector的容量大小；

2、只有当所申请的容量大小n大于vector的当前容量时，才会重新为vector分配存储空间；

3、reserve方法对于vector的大小(即size)没有任何影响；

  

具体通过下面的例子验证

[cpp] [view
plain](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"view plain")
[copy](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"copy")

  1. #include <iostream>
  2. #include <vector>
  3.   4. using namespace std; 
  5.   6. int main() 
  7. { 
  8. vector<int> vect; 
  9.   10. vect.reserve(5); // 调用reserve方法为vect分配容量（即存储空间）
  11.   12. vect.push_back(1); 
  13. vect.push_back(2); 
  14. vect.push_back(3); 
  15. vect.push_back(4); // 插入4个元素
  16.   17. cout << vect.size() << endl; // vect的实际大小(即包含多少元素)
  18. cout << vect.capacity() << endl; // vect的容量大小
  19.   20. return 0; 
  21. } 

  
结果为

![](0.png)  

  

从结果中，就可以很清楚的看到capacity和size的区别。

  

下面看一个综合的例子，可以从中获得很多信息：

[cpp] [view
plain](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"view plain")
[copy](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"copy")

  1. #include <iostream>
  2. #include <vector>
  3.   4. using namespace std; 
  5.   6. int main() 
  7. { 
  8. vector<int> vect; 
  9.   10. vect.reserve(5); 
  11.   12. vect.push_back(1); 
  13. vect.push_back(2); 
  14. vect.push_back(3); 
  15. vect.push_back(4); 
  16.   17. cout << vect.size() << endl; 
  18. cout << vect.capacity() << endl; 
  19.   20. vect.push_back(5); 
  21. vect.push_back(6); // 插入两个元素，此时vect的大小大于之前分配的容量5
  22.   23. cout << "size1 = " << vect.size() << endl; 
  24. cout << "capacity1 = " << vect.capacity() << endl; 
  25.   26. vect.push_back(7); 
  27. vect.push_back(8); // 在插入两个元素，和上面的结果进行对比，会有意外收获
  28. cout << "size1_1 = " << vect.size() << endl; 
  29. cout << "capacity1_1 = " << vect.capacity() << endl; 
  30.   31. vect.reserve(3); // 当程序执行到此处时，vect的容量大小一定是大于3的
  32.   33. cout << "size2 = " << vect.size() << endl; 
  34. cout << "capacity2 = " << vect.capacity() << endl; 
  35.   36. vect.reserve(12); 
  37.   38. cout << "size3 = " << vect.size() << endl; 
  39. cout << "capacity3 = " << vect.capacity() << endl; 
  40.   41. return 0; 
  42. } 

执行结果为：

![](1.png)  

  

对这一执行结果，一点点进行分析:

1、首先，看结果size1和capacity1，在打印这两个结果前，程序向vect中插入了两个元素，之前，vect中存在4个元素且容量为5，按照我之前的设想，如果我采用push_back向vect中插入元素时，当元素数量大小capacity时，vect的capacity会随着size变大而变大，但应该是和size相等。但此处，vect的大小为6，但是容量却是7，且这个7，来的很是突然，我往哪个方面靠都靠不上啊。好吧，先把这个疑问姑且放下，现在我们猜想，是不是push_back中针对这种情况会有处理，始终保持vect的capacity比size至少大1，带着这个猜想继续向下看，我又向vect中插入了两个元素，此时，vect的大小为8，若是我们刚才的猜想是正确的话，则此时，vect的capacity应该增大为9了，但是此时结果size1_1和capacity1_1却给了我当头一棒，这个10又是怎么回事？学习编程永远记住一件事情，所有问题的答案，都能从代码中找到。于是，我顺着vector的push_back源码开始找下去，就有了下面这段追踪代码

[cpp] [view
plain](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"view plain")
[copy](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"copy")

  1. ... 
  2. void push_back(_Ty&& _Val) 
  3. ... 
  4. if (this->_Mylast == this->_Myend) 
  5. _Reserve(1); 
  6. ... 
  7.   8. void _Reserve(size_type _Count) 
  9. { // ensure room for _Count new elements, grow exponentially
  10. size_type _Size = size(); 
  11. if (max_size() - _Count < _Size) 
  12. _Xlen(); 
  13. else if ((_Size += _Count) <= capacity()) 
  14. ; 
  15. else
  16. reserve(_Grow_to(_Size)); 
  17. } 
  18.   19. size_type _Grow_to(size_type _Count) const
  20. { // grow by 50% or at least to _Count
  21. size_type _Capacity = capacity(); 
  22.   23. _Capacity = max_size() - _Capacity / 2 < _Capacity 
  24. ? 0 : _Capacity + _Capacity / 2; // try to grow by 50%
  25. if (_Capacity < _Count) 
  26. _Capacity = _Count; 
  27. return (_Capacity); 
  28. } 

由这段官方实现代码，终于找到了答案，原来在使用push_back向vect中插入元素时，如果当前元素数量大于vector的capacity时，会重新为vector分配存储空间，而分配的原则就是:

原capacity + 原capacity / 2

这样，就解释了上面的7和10两个结果，最初，vect的容量大小为5，在第一次插入两个元素后，vector中的元素数量大于5了，所以此时，会重新为vect分配容量，分配大小为5
+ 5 / 2 = 7，而同样的，第二次重新分配vect的容量是7 + 7 / 2 = 10，这就合理的解决了刚才的疑问。

  

2、当vect的容量大小为10时，再调用reserve方法，重新为其设置容量为3时，不会进行任何操作，可以看到，vect的容量大小还是10;

3、当为vect设置容量大小为12时，可以看到，成功的改变了vect的容量大小；

4、不论哪次调用reserve方法，vect的size大小在调用前后，始终没有被改变过。

  

到此，是我对reserve方法的一些思考和验证。

  

cplusplus.com中对resize的定义是：  

Resizes the container so that it contains n elements.  
  
If n is smaller than the current container
[size](http://www.cplusplus.com/vector::size), the content is reduced to its
first n elements, removing those beyond (and destroying them).  
  
If n is greater than the current container
[size](http://www.cplusplus.com/vector::size), the content is expanded by
inserting at the end as many elements as needed to reach a size of n. If val
is specified, the new elements are initialized as copies of val, otherwise,
they are value-initialized.  
  
If n is also greater than the current container
[capacity](http://www.cplusplus.com/vector::capacity), an automatic
reallocation of the allocated storage space takes place.  
  
Notice that this function changes the actual content of the container by
inserting or erasing elements from it.  

  

从上述说明中，可以得到下面的信息：

1、resize方法被用来改变vector的大小，即vector中元素的数量，我们可以说，resize方法改变了容器的大小，且创建了容器中的对象；

2、如果resize中所指定的n小于vector中当前的元素数量，则会删除vector中多于n的元素，使vector得大小变为n；

3、如果所指定的n大于vector中当前的元素数量，则会在vector当前的尾部插入适量的元素，使得vector的大小变为n，在这里，如果为resize方法指定了第二个参数，则会把后插入的元素值初始化为该指定值，如果没有为resize指定第二个参数，则会把新插入的元素初始化为默认的初始值；

4、如果resize所指定的n不仅大于vector中当前的元素数量，还大于vector当前的capacity容量值时，则会自动为vector重新分配存储空间；

  

还是通过代码来说话：

[cpp] [view
plain](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"view plain")
[copy](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"copy")

  1. #include <iostream>
  2. #include <vector>
  3.   4. using namespace std; 
  5.   6. int main() 
  7. { 
  8. vector<int> vect; 
  9. int i = 0; 
  10.   11. vect.reserve(10); 
  12.   13. vect.push_back(1); 
  14. vect.push_back(2); 
  15. vect.push_back(3); 
  16. vect.push_back(4); 
  17. vect.push_back(5); 
  18. vect.push_back(6); 
  19. vect.push_back(7); 
  20. vect.push_back(8); // 此时vect的size大小为8
  21.   22. cout << vect.size() << endl; 
  23. cout << vect.capacity() << endl; 
  24.   25. vect.resize(6); // 此处设置vect的大小比当前vect中元素数量小，且没有指定初始化值
  26.   27. cout << "size1 = " << vect.size() << endl; 
  28. cout << "capacity1 = " << vect.capacity() << endl; 
  29.   30. for (i = 0 ; i < vect.size(); i++) 
  31. { 
  32. cout << vect[i] << endl; 
  33. } 
  34.   35. vect.resize(4, 10); // 此处设置vect的大小为4，比前面的6小，且指定了初始化值，看是否会改变前四个元素的值
  36.   37. cout << "size1_1 = " << vect.size() << endl; 
  38. cout << "capacity1_1 = " << vect.capacity() << endl; 
  39.   40. for (i = 0 ; i < vect.size(); i++) 
  41. { 
  42. cout << vect[i] << endl; 
  43. } 
  44.   45. vect.resize(8, 7); // 此处设置vect的大小为8，大于当前vect的大小4，但是小于vect的当前容量10，指定初始化值为7
  46.   47. cout << "size2= " << vect.size() << endl; 
  48.   49. cout << "capacity2 = " << vect.capacity() << endl; 
  50.   51. for (int i = 0 ; i < vect.size(); i++) 
  52. { 
  53. cout << vect[i] << endl; 
  54. } 
  55.   56. vect.resize(10); // 此处设置vect的大小为10，大于当前vect的大小8，但是等于vect的当前容量10，没有指定初始化值，采用默认值
  57.   58. cout << "size3 = " << vect.size() << endl; 
  59. cout << "capacity3 = " << vect.capacity() << endl; 
  60.   61. for (int i = 0; i < vect.size(); i++) 
  62. { 
  63. cout << vect[i] << endl; 
  64. } 
  65.   66. vect.resize(12, 77); // 此处设置vect的大小为10，不仅大于当前vect的大小10，还大于vect的当前容量10，会为vect重新分配存储空间
  67.   68. cout << "size4 = " << vect.size() << endl; 
  69. cout << "capacity4 = " << vect.capacity() << endl; 
  70.   71. for (int i = 0; i < vect.size(); i++) 
  72. { 
  73. cout << vect[i] << endl; 
  74. } 
  75.   76. return 0; 
  77. } 

  
上述代码的执行结果为：

![](2.png)  

  
由上面的执行结果，可以得到下面几个结论：

1、使用resize方法时，改变了vector中元素的个数，即vector的大小，但不会改变vector的容量大小；

2、验证了“如果resize中所指定的n小于vector中当前的元素数量，则会删除vector中多于n的元素，使vector得大小变为n；”，由size1、capacity1、size1_1、capacity1_1，可知，如果resize指定的大小n小于vector当前的大小时，会减小vector的大小，但无论resize中是否指定初始化值，都不会影响vector中原本已经存在的元素值；

3、当resize中所指定的n大于vector当前的大小，但是小于vector当前的容量大小时，会在vector后面插入适量的元素，使得vector的大小满足n，如果指定了初始值，则会把新插入的元素初始化为指定的初始值，如果没有指定初始值，则将会把新插入的元素初始化为默认初始值，即0；(由size2、capacity2、size3、capacity3得出)

4、当resize中所指定的n大于vector当前的大小，并且大于vector当前的容量大小时，会为vector重新分配存储空间，由size4和capacity4可以看出来，其中，对于capacity
= 15的原因上面已经解释过，这里不再赘述。

  

这里再补充一点：

reserve被用来为vector设置容量大小，但是并没有创建vector中的元素对象，必须保证vector中有通过push_back或者insert等方法插入的元素后，才能访问vector中的元素；而resize被用来设置vector的大小，即元素个数，同时会创建元素对象，因此可以通过重载标识符[]来访问vector中resize所指定的大小范围内的元素，若是采用push_back或者insert方法向vector中插入元素的话，则只会在vector的尾部插入新的元素，增加vector的元素个数，扩大vector的大小

  

通过下述代码进行验证：

1、

[cpp] [view
plain](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"view plain")
[copy](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"copy")

  1. #include <iostream>
  2. #include <vector>
  3.   4. using namespace std; 
  5.   6. int main() 
  7. { 
  8. vector<int> vect; 
  9. int i = 0; 
  10.   11. vect.reserve(10); 
  12.   13. cout << vect.size() << endl; 
  14. cout << vect.capacity() << endl; 
  15.   16. vect[0] = 3; 
  17.   18. return 0; 
  19. } 

  

首先，在使用reserve为vect设置容量大小之后，通过[]来访问
vect中的第一个元素，此时，在编译链接过程中，不会有任何问题，但是在执行的过程中会报出：

![](3.png)  

  

该异常信息为：vector subscript out of
range，即访问vect越界了，由此可以看出，在使用reserve时并没有为vector创建任何元素对象

  

2、

[cpp] [view
plain](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"view plain")
[copy](https://blog.csdn.net/vampirem_chosen_one/article/details/50519870#
"copy")

  1. #include <iostream>
  2. #include <vector>
  3.   4. using namespace std; 
  5.   6. int main() 
  7. { 
  8. vector<int> vect; 
  9. int i = 0; 
  10.   11. vect.resize(6); 
  12.   13. cout << "size1 = " << vect.size() << endl; 
  14. cout << "capacity1 = " << vect.capacity() << endl; 
  15.   16. vect[5] = 17; 
  17.   18. for (i = 0 ; i < vect.size(); i++) 
  19. { 
  20. cout << vect[i] << endl; 
  21. } 
  22.   23. vect.push_back(777); 
  24.   25. cout << "size1 = " << vect.size() << endl; 
  26. cout << "capacity1 = " << vect.capacity() << endl; 
  27.   28. for (i = 0 ; i < vect.size(); i++) 
  29. { 
  30. cout << vect[i] << endl; 
  31. } 
  32.   33. return 0; 
  34. } 

  
上述代码的执行结果为：

![](4.png)  

  

由上述执行结果可以看出，在使用resize为vector指定大小之后，会创建元素对象，可以通过[]标识符来对元素对象进行访问，同时若是采用push_back或者insert等方法来插入元素时，实际上会在vector尾部插入新的元素，从上面的size1值的改变可以证实。

  

本文为VampirEM原创博文，如需转载，请注明出处！

