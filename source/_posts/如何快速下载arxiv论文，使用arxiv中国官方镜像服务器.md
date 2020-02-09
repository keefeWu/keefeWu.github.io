---
title: 如何快速下载arxiv论文，使用arxiv中国官方镜像服务器
date: 2020-02-08 15:18:26
tags:
---
转载自 [简书](https://www.jianshu.com/p/184799230f20)

简单的说，就是arxiv 在中国有官方镜像
[http://cn.arxiv.org](https://link.jianshu.com?t=http://cn.arxiv.org/)，通过使用
chrome 插件将 arxiv
的链接自动重定向到中国镜像网站链接即可，这样当你点击一篇文章的arxiv链接后就可以自动到cn.arxiv.org，速度很快。

### 背景

arXiv
（[https://arxiv.org/](https://link.jianshu.com?t=https://arxiv.org/)）是一个收集计算机科学、物理学、数学和生物学等多个学科的论文预印本网站，主站点在康奈尔大学，在全球多个地方设置有镜像网站。  
对于深度学习专业，可以说绝大多数论文都是从 arxiv 上获取的，因此能够快速的访问 arxiv 非常重要。  
由于国内多方面原因导致经常直接访问速度比较慢，因此可以通过使用在中国区的镜像站点（[http://cn.arxiv.org](https://link.jianshu.com?t=http://cn.arxiv.org/)
，由中科院理论物理所支持）来加速。但是通常在其他地方查询到 arxiv 链接，如果每次手动修改网址比较麻烦。因此这里推荐采用以下方法来解决。

### 解决方案

chrome 插件 tampermonkey（油猴插件） 是一款功能强大的脚本插件，可以通过脚本对浏览器上网页进行修改编辑等，更多介绍可以参考
[https://zhuanlan.zhihu.com/p/28869740](https://link.jianshu.com?t=https://zhuanlan.zhihu.com/p/28869740)  
因此，这里我们使用该插件对网页中的arxiv 链接进行重定向到 cn.arxiv.org

  1. 安装chrome 浏览器。推荐使用google chrome官方下载地址 ；如果无法访问，使用百度下载也可以。
  2. 安装tempermonkey插件，推荐使用 chrome webstore 官方网址；如果无法下载，在 crx4chrome 网站搜索并下载也可以，这里给出crx4chrome网站上tampermonkey插件的[下载链接](https://link.jianshu.com?t=https://www.crx4chrome.com/down/755/crx/)。
  3. 添加 arxiv 重定向脚本。  
代码更新时间2017年12年04日，自动转到pdf链接。  
代码需要全部复制粘贴，部分看似注释的代码也有用处，代码如下

    
    
    // ==UserScript==
    // @name        Redirect arxiv.org to CN.arxiv.org/pdf
    // @namespace   uso2usom
    // @description On any web page it will check if the clicked links goes to arxiv.org. If so, the link will be rewritten to point to cn.arxiv.org
    // @include     http://*.*
    // @include     https://*.*
    // @version     1.2
    // @grant       none
    // ==/UserScript==
    
    // This is a slightly brute force solution, but there is no other way to do it using only a userscript.
    
    // Release Notes
    
    // version 1.2
    // Focus on pdf link only!
    // Add '.pdf' link  automatically. Convenient for saving as pdf.
    
    // version 1.1
    // Redirect arxiv.org to CN.arxiv.org
    
    document.body.addEventListener('mousedown', function(e){
        var targ = e.target || e.srcElement;
        if ( targ && targ.href && targ.href.match(/https?:\/\/arxiv.org\/pdf/) ) {
            targ.href = targ.href.replace(/https?:\/\/arxiv\.org/, 'http://cn.arxiv.org');
        }
        if ( targ && targ.href && targ.href.match(/http?:\/\/arxiv.org\/pdf/) ) {
            targ.href = targ.href.replace(/http?:\/\/arxiv\.org/, 'http://cn.arxiv.org');
        }
        if ( targ && targ.href && targ.href.match(/https?:\/\/arxiv.org\/abs/) ) {
            targ.href = targ.href.replace(/https?:\/\/arxiv\.org\/abs/, 'http://cn.arxiv.org/pdf');
        }
        if ( targ && targ.href && targ.href.match(/http?:\/\/arxiv.org\/abs/) ) {
            targ.href = targ.href.replace(/http?:\/\/arxiv\.org\/abs/, 'http://cn.arxiv.org/pdf');
        }
        if (targ && targ.href && targ.href.match(/http?:\/\/cn.arxiv.org\/pdf/) && !targ.href.match(/\.pdf/) )
        {
           targ.href = targ.href + '.pdf';
        }
    });
    
    

  4. 测试配置是否成功，下面是arxiv上的一篇文章作为示例，点击看网址前面是否已经加上“cn.”前缀，点击pdf测试速度。该文章共57页，之后可以手动去掉“cn.”前缀对比速度。  
[NIPS 2016 Tutorial: Generative Adversarial
Networks](https://link.jianshu.com?t=arxiv.org/abs/1701.00160)

  5. 说明  
由于 [http://cn.arxiv.org](https://link.jianshu.com?t=http://cn.arxiv.org/)
并不是主站点，是 arxiv 在中国区的镜像，因此更新有大约半天的延迟，对于当天提交的文章，可能更新不及时。对于当天文章可以手动删除“cn.”前缀解决。  
如果出现 pdf 正在自动从源文件生成等提示，为正常现象，稍后即可获取pdf论文。

  
  
作者：德谟赛斯  
链接：https://www.jianshu.com/p/184799230f20  
來源：简书  
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。

