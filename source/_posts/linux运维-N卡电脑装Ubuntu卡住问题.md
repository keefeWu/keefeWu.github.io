---
title: linux运维-N卡电脑装Ubuntu卡住问题
date: 2020-02-08 15:11:19
tags:
---
因为ubuntu识别出N卡之后会自动装一个错误的驱动，所以运气好的能够登录，卡在桌面，运气不好的直接无法进入。那么怎么办呢？  
首先，在启动的时候狂敲`shift`直到能够进入那个选择ubuntu安装还是进入的那个菜单，然后按`e`进入临时grub编辑界面，（记住是临时的，也就是说这次没弄好，下次重启又要改一次），然后找到`quiet
splash`，在后面添加`acpi_osi=linux nomodeset`，然后按`F10`保存开机。  
进去之后呢就要装显卡驱动了，这个时候是不能有桌面显示的，所以先进入命令行模式

    
    
            ctrl+alt+F1
    

然后结束掉桌面

    
    
           sudo service lightdm stop
    

然后安装驱动

    
    
            sudo add-apt-repository ppa:graphics-drivers/ppa
            ubuntu-drivers devices
            sudo apt-get install nvidia-[version]
    

然后恢复桌面

    
    
           sudo service lightdm start
    

这个时候应该就好了，如果还不好就重启看看

