---
title: git-pro笔记-《Git-基础》
date: 2020-02-08 15:17:40
tags:
---

    git state

会显示当前在哪个分支，哪些文件修改了没有添加到暂存区，哪些添加了没有提交。

    
    
    git add readme.md

就会把一些文件添加到暂存区(stage)了

这个时候再运行

    
    
    git state

就会显示readme.md是newfile状态了

如果没有add就会告诉你它是modified状态，会问你要不要添加这个文件到缓存区

如果add完之后你又修改了这个文件，再使用state会出现两个状态

new file 和 modified

其中new file就是你当前这个没add的文件，而modified的文件实际上是你上次add放进缓存区里的

  

因为git add是可以添加目录的，一般用

    
    
    git add .

添加当前目录下所有的文件以及文件夹，如果想忽略一些，那么就要建立一个.gitignore的文件

然后写上想忽略的文件名，像这样

    
    
    *.[oa]
        *~

第一行告诉 Git 忽略所有以 `.o` 或 `.a` 结尾的文件。一般这类对象文件和存档文件都是编译过程中出现的，我们用不着跟踪它们的版本。第二行告诉
Git 忽略所有以波浪符（`~`）结尾的文件，许多文本编辑软件（比如 Emacs）都用这样的文件名保存副本。此外，你可能还需要忽略 `log`，`tmp`
或者 `pid` 目录，以及自动生成的文档等等。要养成一开始就设置好 `.gitignore` 文件的习惯，以免将来误提交这类无用的文件。

文件 `.gitignore` 的格式规范如下：

  * 所有空行或者以注释符号 `＃` 开头的行都会被 Git 忽略。
  * 可以使用标准的 glob 模式匹配。
  * 匹配模式最后跟反斜杠（`/`）说明要忽略的是目录。
  * 要忽略指定模式以外的文件或目录，可以在模式前加上惊叹号（`!`）取反。

  

如果想查看已经修改了的文件和暂存区文件的区别，可以使用

    
    
    git diff

进行查看，注意这里是指的修改了还没add的文件和暂存区的区别，如果要查看暂存区和工作空间（上次提交）的区别，则使用一个参数

    
    
    git diff --cached

新版的使用

    
    
     git diff --staged

就可以了。

  

一切都添加好了以后，使用

    
    
    git commit -m "Story 182: Fix benchmarks for speed"

进行对更改进行提交，从暂存区提交到了工作空间，并且添加了一条说明

如果直接使用

    
    
    git commit

则会自动打开git的文本编辑器，进行输入，并且附带了各种修改信息的注释，在关闭编辑器以后自动删除那些注释

还有一种不用add直接提交的办法，就是

    
    
    git commit -a

这样就跳过了暂存区，而是直接执行

    
    
    git add . git commit

这两条操作了

如果之前提交的一次感觉有漏掉的，想要删除那次提交，可以使用

    
    
    git commit --amend

重新提交

如果add到暂存区的文件又不想要了，可以使用

    
    
    git reset HEAD benchmarks.rb

撤销掉刚才添加的这个benckmarks.rb文件

如果想要删除某个文件，则执行

    
    
    git rm grit.gemspec

那么这个文件就会从工作空间中被删除掉，如果之前修改过这个文件，并且放到暂存区了的话，则需要使用-f参数来强制删除，这样防止新的修改没有记录的误删操作

  

git 自己有一个递归操作符号，就是反斜杠\，例如

    
    
    git rm log/\*.log

这一句就是删除log目录以及其子目录下所有后缀为log的文件

  

移动文件

git的移动文件和shell操作很像

    
    
    git mv file_from file_to

就是把file_from改名成了file_to

其实相当于执行了

    
    
    git rm file_from git add file_to

两步操作

  

使用

    
    
    git log

可以看到所有的提交历史以及作者信息

加上参数

    
    
    git log -p -2

则可以看到近两次提交详细的变化情况

其中-p是变化，-2是显示近2次内容

  

如果修改了某个文件，还没有添加到暂存区，现在又不想要了，想要回到上次提交的状态，这个时候使用

    
    
    git checkout -- benchmarks.rb

把benckmarks.rb恢复到工作空间中上次提交的状态。

这个操作相当危险，因为现在修改的内容并没有记录到工作空间里，所以一旦执行了这条命令这次修改的内容将在git中无法找到了

  

 **远程操作**

任何从远程克隆下来的项目，都会有至少一个远程仓库，默认是origin，我们使用

    
    
    git remote

  

可以查看所有的远程仓库

如果加上参数-v则可以看到远程仓库对应的地址

如果想要添加一个新的远程仓库，只需要

    
    
    git remote add pb git://github.com/paulboone/ticgit.git

就可以把这个地址对应的仓库添加到这个工程里了，并且给他取了一个名字叫pb，这个名字是自己随便取的，和远程完全无关，方便我们接下来使用这个名字快速的操作这个仓库

如果想要下载这个远程仓库的所有内容，我们使用

    
    
    git fetch pb

在本地就可以看到一个叫做pb/master的分支了，这个分支是独立的，不会影响到本地之前已有的分支，如果想要它影响到本地的分支，需要在之后再进行一个merge操作

使用

    
    
    git remote show origin

可以看到某一个分支的详细信息，甚至包括了如果执行缺省的push会是怎样的操作

还有远程有哪些分支，哪些分支没有同步到本地

  

如果想把远程的仓库重命名，可以使用

    
    
    git remote rename pb paul

这样就把pb这个远程仓库改名成了paul

如果想要直接删除某个远程仓库，可以使用

    
    
    git remote rm paul

  

