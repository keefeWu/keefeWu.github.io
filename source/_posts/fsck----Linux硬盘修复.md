有时候硬盘由于强制关机之类的操作造成了一定的损坏，造成部分分区挂载不上，我们可以使用fsck来修复

全称 file system consistency check，是linux自带的一个很强大的工具，基本上强行关机造成的损坏都可以修复

用法如下：

首先使用一些disk查看工具，看看自己要修复的分区是哪一个

比方说是sdc3那么就执行

sudo fsck /dev/sdc3  

就可以了

之后如果坏了会问你某某坏了，是否需要修复，点击回车yes就行了

当然，前提是硬盘要处于卸载状态，不能挂载才能执行这个命令。

