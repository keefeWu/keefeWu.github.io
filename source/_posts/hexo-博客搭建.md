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
<code>
backup:
&emsp;&emsp;type: git
&emsp;&emsp;repository:
&emsp;&emsp;&emsp;&emsp;github: git@github.com:xxx/xxx.git,branchName
</code>
之后每次用
<code>hexo b</code>
就可以备份上传了
注意一点，这里的分支一定要和master分支区分开，master就专门用来存放网站的，源码放在另一个分支上，可以新建一个source分支，然后把source分支设置成主分支，这样的话下次你<code>git clone</code>就可以直接clone代码了。
## 远程配置
这玩意自动关联的git，所以需要设置一下，也是在_config.yml中，它本来就有<code>deploy</code>,所以补充一下就行了
<code>
deploy:
&emsp;&emsp;type: git
&emsp;&emsp;repo: git@github.com:xxx/xxx.git
&emsp;&emsp;branch: master
</code>
之后使用<code>hexo d</code>就是上传到git了，不过也需要安装一个插件
<code>npm install hexo-deployer-git --save</code>

## 基础操作
以上配置好了之后实际上就可以用了，如果要新建文章则使用<code>hexo new 文章名</code>,注意这里文章名如果有空格的话记得用斜杠转译，就像我这篇文章的标题一样，是用的<code> hexo new hexo\ 博客搭建</code>
新建好的文件在source这个目录里，其实你输入了new之后命令行打印出来了文件路径，就是那个md文件，markdown格式的，去了修改就行了。
写好之后渲染成html的，使用<code>hexo g</code>也就是generator渲染，如果要本地测试的话就是<code>hexo s</code>，开启一个server，提交到git就用<code>hexo d</code> deploy，上传源码就使用<code>hexo b</code> backup

## hexo环境搭建
我把这个放到最后，是因为这个的资料太多了，前面几个都是我个人总结的，比较难查到的，这个是想着做事做到尽善尽美才加上的。
首先下载nodejs，因为hexo是用node写的，地址在https://nodejs.org/en/
解压之后把这些放到/usr/local下面就行了
然后node的安装源可以换成国内的加速
<code>npm config set registry https://registry.npm.taobao.org</code>
当然，这个很显然是可有可无的
然后是安装hexo了
<code>npm i hexo-cli -g</code>
这样就算部署完毕了

## 更换主题
这个我觉得挺重要，默认的那个landscape的主题确实太土了，所以可以去主题仓库找找看https://hexo.io/themes/

找到满意的之后进入theme目录，然后把主题clone下来
比方说我这个主题
<code>git clone https://github.com/fi3ework/hexo-theme-archer.git</code>
下好之后在_config.yml中配置一下theme，就是目录名就可以了。

## 统计访问量
我这个主题是自带统计访问量的，但是需要安装一个插件，所以需要执行
<code>npm i --save hexo-wordcount</code>
然后修改在 archer 的配置中的
<code>reading_info: true</code>
不过busuanzi这个js偷偷的把pv、uv的内容隐藏了，就是加上了<code>“display: none;”</code>属性，我在网上看见有人说需要人工去打开，就是在这个主题的css里，我这个主题是在_intro.scss文件里面，加上了
```
#busuanzi_container_site_pv {
    /* 网站页脚访问pv */
    display: inline !important;
}

#busuanzi_container_site_uv  {
    /* 网站页脚访问uv */
    display: inline !important;
}

#busuanzi_container_page_pv {
    /* 文章访问pv */
    display: inline !important;
}
```
但是呢。我这个加了还是有，没办法，我只好去 "base-footer.ejs"文件里面把pv的id去掉，变成
```
  <% if(theme.busuanzi_pv_or_uv === 'pv') { %> 
    <span><%- sloganPieces[0] %><span id="busuanzi_value_site_pv"></span><%- sloganPieces[1] %></span>
    <% } else if (theme.busuanzi_pv_or_uv === 'uv') { %>
    <span><%- sloganPieces[0] %><span id="busuanzi_value_site_uv"></span><%- sloganPieces[1] %></span>
    <% } %>
    </div>
    <% } %>
    <script async src="//busuanzi.ibruce.info/busuanzi/2.3/busuanzi.pure.mini.js"></script>
```