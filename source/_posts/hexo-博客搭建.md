---
title: hexo 博客搭建
date: 2020-01-01 10:42:40
tags:
---
## 备份
因为hexo提交到git上只有生成的html文件，没有源码文件，导致一旦换了电脑就直接gg了，所以我找了一个插件
[GitHub](https://github.com/coneycode/hexo-git-backup)
### 使用方法
<code>npm install hexo-git-backup --save</code>

在_config.yml中添加一项

"""
backup:
    type: git
    repository:
       github: git@github.com:xxx/xxx.git,branchName
       gitcafe: git@github.com:xxx/xxx.git,branchName
"""