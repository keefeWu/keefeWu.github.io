c++标准流读取txt文件，虽然很容易搜索到，但是由于很常用，我还是记录在自己的博客里。

首先是头文件

    
    
    #include <iostream>
    #include <fstream>
    #include <string>

第一个是输入输出流，因为接下来文件读取和写入都是用的这个流，也就是>>和<<

第二个就是包含了文件读取的类的头文件

第三个是字符串的头文件，因为都出来首先是字符串格式的

接下来读取的代码

    
    
        std::ifstream fin(data_file);
        if(!fin)
        {
            printf("Read data from file faild, check if the file\n\
            %s\nexist",data_file);
            return false;
        }
        string s;  
        while( fin >> s ) 
        {    
            cout << "Read from file: " << s << endl;  
        }
    
    
    

创建一个ifstream的对象，输入的是文件路径

然后那个对象fin可以直接使用if语句检测是否读取成功，如果返回是false那么就是没成功

然后loop这个对象，不断的使用>>搜刮它，它会根据空格、回车、以及逗号来分割字符，直到分割完成，它被取空了又会变成false

使用>>传递给string类型的对象，具体想转成数字可以使用std::atoi() std::atof()等等转成数字类型。

