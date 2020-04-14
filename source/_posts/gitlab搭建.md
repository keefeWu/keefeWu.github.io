---
title: gitlab搭建
date: 2020-04-14 10:44:02
tags:
---

```
export GITLAB_HOME=你想要挂载数据的目录
sudo docker stop gitlab;
sudo docker rm gitlab;
sudo docker run --detach \
  --hostname 机器的ip或者域名注意后面还有那个冒号: \
  --publish 443:443 --publish 80:80 --publish 22222:22 \
  --name gitlab \
  --restart always \
  --volume $GITLAB_HOME/gitlab/config:/etc/gitlab \
  --volume $GITLAB_HOME/gitlab/logs:/var/log/gitlab \
  --volume $GITLAB_HOME/gitlab/data:/var/opt/gitlab \
store/gitlab/gitlab-ce:11.10.4-ce.0                                      
```

因为绑定了22端口映射的是22222端口，所以需要改一下配置，先运行，让他生成config/gitlab.rb文件，然后使用sudo权限编辑，
取消如下几个注释
```
gitlab_rails['gravatar_plain_url'] = 'http://www.gravatar.com/avatar/%{hash}?s=%{size}&d=identicon'
```
这个是可以自定义头像的
```
gitlab_rails['gitlab_shell_ssh_port'] = 22222
```
这个就是改变22端口到22222