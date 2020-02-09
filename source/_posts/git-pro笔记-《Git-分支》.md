---
title: git-pro笔记-《Git-分支》
date: 2020-02-08 15:17:36
tags:
---
如果想要创建一个叫做testing的分支，则使用  
`git branch testing`  
不过执行完之后只是创建了一个分支，但是并没有切换到testing分支去，如果想切换过去，执行  
`git checkout testing`  
我觉得分支的作用也是分支这一节的重点，所以我把git pro中的话也摘录下来了

> *现在让我们来看一个简单的分支与合并的例子，实际工作中大体也会用到这样的工作流程： 开发某个网站。 为实现某个新的需求，创建一个分支。
> 在这个分支上开展工作。 假设此时，你突然接到一个电话说有个很严重的问题需要紧急修补，那么可以按照下面的方式处理：  
>  返回到原先已经发布到生产服务器上的分支。 为这次紧急修补建立一个新分支，并在其中修复问题。  
>  通过测试后，回到生产服务器所在的分支，将修补分支合并进来，然后再推送到生产服务器上。 切换到之前实现新需求的分支，继续工作。*

使用`checkout`再加上一个参数也能达到创建分支的作用，例如  
`git checkout -b iss53`  
就是创建一个名字叫iss53的分支，并且切换进去

当这个分支的修改全部完成了，想要将它发布到master分支，这时候就要使用merge操作了  
首先切回master分支  
`git checkout master`

然后进行merge操作  
`git merge iss53`  
此时iss53分支完成了使命，可以删除掉了，那么使用  
`git branch -d iss53`

如果你两个分支同时修改了同一个文件中的同一个部分，就会形成冲突，使用merge操作git不会提交合并，而是会提示冲突等你解决。例如：

> $ git merge iss53  
>  Auto-merging index.html  
>  CONFLICT (content): Merge conflict in index.html  
>  Automatic merge failed; fix conflicts and then commit the result.

这个时候是可以用`git status`来查看

> [master*]$ git status  
>  index.html: needs merge  
>  # On branch master  
>  # Changes not staged for commit:  
>  # (use “git add …” to update what will be committed)  
>  # (use “git checkout – …” to discard changes in working directory)  
>  #  
>  # unmerged: index.html  
>  #

可以看到，是index.html这个文件出了问题，那么我们打开这个文件，git其实把冲突部分都用明显符号标记出来了

手动解决完毕之后 ，再把这个文件add进去，然后提交。

使用`git branch`查看所有的分支，其中当前分支名字前面会带上一个`*`  
如果加上一个参数`git branch -v`可以查看，每个分支最新一次提交的详细信息

使用`git branch --merged`可以查看哪些分支是哪些分支的上游，如果是最上游的分支都会带上` __`,这就意味着如果没有带`
`的分支都可以被删掉，因为他们被某一个带` * `的分支完全包含了，实际上并没有什么用处了。  
同样，使用  
`git branch --no-merged`能够查看和merge毫无关系的分支  
不过这些分支没有任何上游保护，所以乱删除很危险，直接使用`git branch -d `会提示错误，如果非要删除，则使用`git branch
-D`进行强制删除

如果本地一切都更新好了，就可以推送到服务器了，操作是这样的  
git push (远程仓库名) (分支名) 例如：  
` git push origin serverfix `  
这样就是推送了一个serverfix分支到origin服务器上了，当然，完整的写法应该是  
` git push origin serverfix:serverfix `  
意思就是把本地的serverfix 提交到origin服务器的serverfix分支上，还可以改一个名字，例如：  
`git push origin serverfix:awesomebranch`  
这个就是说把本地的serverfix 提交到origin服务器的awesomebranch分支上

可以使用  
`git fetch origin`  
来下载远程分支，这样你的本地就有一个  
origin/awesomebranch这个分支了，但是你无法修改它，如果想把它合并到当前分支，直接使用

`git merge origin/awesomebranch`  
这个命令来合并  
如果想在本地也建立一个awesomebranch来进行开发，可以使用  
`git checkout -b awesomebranch origin/awesomebranch`  
来复制一个本地的awesomebranch分支

像这种方法直接从远程复制出来的分支，能够和远程保持直接的关联，叫做 **跟踪分支 (tracking branch)**  
_在跟踪分支里输入 git push，Git 会自行推断应该向哪个服务器的哪个分支推送数据。同样，在这些分支里运行 git pull
会获取所有远程索引，并把它们的数据都合并到本地分支中来。_

如果想自己关联两个分支，可以使用`track`的方法  
`git checkout --track origin/serverfix`  
这样就是把当前分支和远程的serverfix分支建立了关联，当然也可以指定另一个分支  
`git checkout -b sf origin/serverfix`  
这样可以把sf分支和远程的serverfix分支建立起联系

删除远程分支的语法非常奇怪，是这样的`git push [远程名] :[分支名]`  
例如：  
`git push origin :serverfix`  
当然，git pro给了一个解释，我们可以参考一下

> 咚！服务器上的分支没了。你最好特别留心这一页，因为你一定会用到那个命令，而且你很可能会忘掉它的语法。有种方便记忆这条命令的方法：记住我们不久前见过的
> git push [远程名] [本地分支]:[远程分支] 语法，如果省略 [本地分支]，那就等于是在说“在这里提取空白然后把它变成[远程分支]”。

其实这一章的最后还有一个rebase操作的，但是由于这个操作过于复杂，所以我准备单独开一篇文章仔细分析。

