转载自：<https://blog.csdn.net/u014453898/article/details/64124269>

0.static修饰类中成员，表示类的共享数据

1.static类成员

在C++primer里面说过，static类成员不像普通的类数据成员，static类数据成员独立于一切类对象处在。static类数据成员是与类关联的，但不与该类定义的对象有任何关系。这句话什么意思？就是static不会想普通类数据成员一样每一个类对象都有一份，全部类对象是共享一个static类成员的，例如A类对象修改了static成员为1，那么B对象对应的static类对象成员的值也会是1.

注意：static类对象必须要在类外进行初始化

如：

static类对象必须要在类外进行初始化  

[cpp] [view plain](https://blog.csdn.net/u014453898/article/details/64124269#
"view plain")
[copy](https://blog.csdn.net/u014453898/article/details/64124269# "copy")

  1. class Text 
  2. { 
  3. public: 
  4. static int count; 
  5. }; 
  6.   7. int Text::count=0;//用static成员变量必须要初始化 
  8.   9. int main() 
  10. { 
  11. Text t1; 
  12. cout<<t1.count<<endl; 
  13. return 0; 
  14. }//程序输出0

  

所有对象共享一个static类成员

static修饰的变量先于对象存在，所以static修饰的变量要在类外初始化。因为static是所有对象共享的东西嘛，必须要比对象先存在的。

[cpp] [view plain](https://blog.csdn.net/u014453898/article/details/64124269#
"view plain")
[copy](https://blog.csdn.net/u014453898/article/details/64124269# "copy")

  1. class Text 
  2. { 
  3. public: 
  4. static int count; 
  5. }; 
  6.   7. int Text::count=0;//用static成员变量必须要初始化 
  8.   9. int main() 
  10. { 
  11. Text t1; 
  12. Text t2; 
  13.   14. t1.count = 100; //t1对象把static成员count改为100 
  15. cout<<t2.count<<endl; //当t2对象打印static成员的时候，显示的是100而不是0 
  16. return 0; 
  17. } 

好处：

用static修饰的成员变量在对象中是不占内存的，因为他不是跟对象一起在堆或者栈中生成，用static修饰的变量在静态存储区生成的，所以用static修饰一方面的好处是可以节省对象的内存空间。就如同你创建100个Person对象，而这100个对象都有共有的一个变量，例如叫国籍变量，就是Person对象的国籍都是相同的，那如果国籍变量用static修饰的话，即使有100个Person对象，也不会创建100个国籍变量，只需要有一个static修饰的国籍变量就可以了，这100个对象要用时，就会去调用static修饰的国籍变量。否则有100个Person变量，就会创建100个国籍变量，在国籍变量都是相同的情况下，就等于浪费空间了，因为你不需要创建100个国籍变量，

  

2.static类成员函数

由于static修饰的类成员属于类，不属于对象，因此static类成员函数是没有this指针的，this指针是指向本对象的指针。正因为没有this指针，所以static类成员函数

不能访问非static的类成员，只能访问 static修饰的类成员。

[cpp] [view plain](https://blog.csdn.net/u014453898/article/details/64124269#
"view plain")
[copy](https://blog.csdn.net/u014453898/article/details/64124269# "copy")

  1. class Text 
  2. { 
  3. public: 
  4. static int fun() 
  5. { 
  6. return num; 
  7. } 
  8. static int count; 
  9. int num; 
  10. }; 
  11. int Text::count=5;//用static成员变量必须要初始化 
  12.   13. int main() 
  14. { 
  15. Text t1; 
  16. Text t2; 
  17. t1.num=100; 
  18.   19. t1.fun();//发生错误，fun函数return的是非static类成员 如果return count就正确 
  20. return 0; 
  21. } 
  22. 

