---
title: adaboost的通俗理解
date: 2020-02-08 15:22:33
tags:
---
  
  
首先先讲个故事吧，三人成虎的故事应该都听说过吧。就是当三个人都来说同一件事情的时候，假的也成了真的。这个算法其实就是利用的这个原理，首先构造多个决策器，然后由多个决策器共同投票来确定一件事情的真伪。  

下面直接上列子：

![](0.png)  

![]()  

这里有0-9十条数据，这十条数据就是供给我们训练的。首先将这十件事情的权重都设为0.1。我们现在需要确定一个阈值（即分界点），把这十条数据分成误差率最小的两类。

假设我们将阈值设成0（即大于0的为一类，小于等于0的为第二类），那么理想情况应该是左边都是1，右边都是-1（也可以反过来，取错误率低于0.5的一种分法），然而现实是左边都是1满足了，而右边有3、4、5、9四条数据都错了，因为他们权重都是0.1，所以这种分法的错误率是0.4。再看反过来的情况，左边是-1，右边是1的话就有6个分错的，这样错误率就是0.6了，对于0这个阈值来说取错误率小于0.5的分法，所以我们说0作为阈值的错误率是0.4。

同样，我们遍历所有的阈值，可以查询出来当阈值为2或者8的时候错误率最小，为0.3。

我们就取2为第一个决策器的阈值（当然你选8也是可以的，只不过为了写程序便利惯用第一个检索到的）。记住故事上所说的，我们需要多个决策器来共同确定某一件事情的真伪，所以选出这一个决策器自然是不够的，首先他在判断6、7、8这三条数据的时候就出错了。怎样避免在6、7、8这三条上再次犯错呢，我们就需要构造更多的决策器。因为第一个决策器在6、7、8上犯错误了，所以他在最后表决权的权重就是0.4236，这个数值是怎么算来的呢？

我们看原文：

![](2.png)  

![]()  

这里给出了计算公式，其中的et就是指的当前决策器的误差率，也就是0.3，那么代入公式我们可以算出当前决策器的权重，也就是表决权是0.4236。不要问我这个公式是怎么来的，这个自然是伟大的数学家推导出来的最优解法，我们学会使用就可以了。

我们算出了当前决策器的表决权为0.4236，但是他在判断6、7、8的时候会出问题，所以我们在构造下一个决策器的时候就要放大这个错误，让以后的决策器能够重视这个错误，从而避开这个错误。那么怎么放大这个错误呢，自然是增加这三条数据的权重，同时为了让所有数据的权重和为1（即让权重统一标准，这样方便以后的比较，不然每次错误的都增大，最后就像经济泡沫一样总的权重越来越大了，这可不是我们希望的）。那么问题来了，这个权重应该增大到多少呢？我们再来看原文给出的公式：

![](4.png)  

![]()  

这里的D就是每条数据的权重，而α就是这个决策器的表决权，也就是刚才算出的0.4236，h就是指的每一条数据的期望值，y指的实际值，比如第6条数据，我们的期望值是-1，可他的实际值确实1，这样h（6）就是-1，y6就是-1，exp指的是e^x，那么第6条数据新的权值就应该是0.1*(e^(-0.4236))/z，现在唯一不知道的就是这个z了，我们再来看一个正确的数据，比如说第1条，他的新权值就应该是0.1*(e^(0.4236))/z，那么我们这组数据中正确的有7个，错误的有3个，所以总的权值就应该是0.1*(e^(-0.4236))/z*3+0.1*(e^(0.4236))/z*7，并且总和应该是=1的，所以就是（0.15274505*3+0.06546857*7）/z=1，可以算出0.91651514/z=1，z也就是0.91651514。

这个z出现的作用仅仅就是提供一个缩放比例，让他们总和变成1，那么可以算出新的权重为D2 = (0.0715, 0.0715, 0.0715, 0.0715,
0.0715, 0.0715, 0.1666, 0.1666, 0.1666,
0.0715），以上数据都是我自己算出来的，不太清楚的可以照着上面的步骤自己验算一遍。

这样新的权重确定出来了，就要开始选择第二个选择器了，重复之前的检索方法，选出第二个选择器的阈值，然后用现在的权重算出错误率，（记住这次错误率不再用0.1来算了，而是用新的权重计算）。例如根据检索情况可以算出当阈值取8的时候错误率最低，只有3、4、5错了，他们的权值是0.0715，所以算出这个决策器的错误率为0.2143。同样，再带入公式算出这个决策器的表决权为0.6496。然后再次更新各个数据的权值，直到满足一定条件时停止（这个条件可以是生成了多少个决策器，也可以是所有决策器投票的结果全部正确）。

经过多次重复以后，达到了我们想要的条件，我们将这多个决策器的信息整合起来，遇到测试数据的时候就让这多个决策器共同判断，根据他们的权重总和他们的结论，选择投票结果大的结果。

假设我们只选出了这两个决策器，现在给定一个X为3，根据一号决策器判断其y为-1，而二号决策器判断其y为1，此时计算每个决策器的表决权，说-1的占了0.4236，而说1的占了0.6496，所以我们最后确定X为3时y是1，显然这根实际的情况是相反的，造成这一情况的原因是我们的决策器太少了，单个的决策器占得话语权太重了。就好像在联合国美国说好的就是好的，哪怕坏的也是好的一样，为了得到正确答案，我们知道兼听则明，所以需要选择多个决策器，共同表决一件事情。这样就是整个算法的思想。

最后是上代码的时间：

#include "adaboost.h" #include "iostream" #include "math.h" using namespace
std; //创建决策器链表 void CreateDecision(Decision *&decision) { decision=(Decision
*)malloc(sizeof(Decision)); decision->pre=NULL; } //插入新的决策器 void
InsertDecision(Decision *&decision) { Decision *s; s=(Decision
*)malloc(sizeof(Decision)); decision->next=s; s->pre=decision; s->next=NULL;
decision=s; } double calAlpha(double e) { double alpha=log((1-e)/e)/2; return
alpha; } int Adaboost::CalH(int data[],int feature,Decision *decision) { int
h=0; if(decision->direction==0) { if(data[feature]>decision->threshold) { h=1;
} else { h=-1; } } else { if(data[feature]>decision->threshold) { h=-1; } else
{ h=1; } } return h; } void Adaboost::UpdateDataWeight(Data data[],int num,int
feature,Decision *&decision) { int h=0; for(int i=0;i<num i=""
h="CalH(data[i].X,feature,decision);" data="" i=""
weight="data[i].WEIGHT*exp(-1*decision-">weight*data[i].Y*h); } double
z=0;//归一化 for(int i=0;i<num;i++) { z+=data[i].WEIGHT; } for(int i=0;i<num;i++)
{ data[i].WEIGHT=data[i].WEIGHT/z; cout<<data[i].WEIGHT<<endl; } } void
Adaboost::ThreAndWeight(Data data[],int num,int feature,Decision *&decision) {
int threshold=0;//定义阈值 double error=0;//定义当前错误率 double min_error=1;//记录最小的错误率
int i_thre;//记录最佳阈值的位置 int direction=0;//定义大小方向，0型为左边-1，1型为左边1 //选出最佳阈值
for(int i=0;i<num;i++) { threshold=data[i].X[feature];//使当前阈值为第i个数据的X
//统计小于阈值部分的错误率 for(int j=0;j<num;j++) { //默认左边为-1，右边为1
if(data[j].X[feature]<=threshold) { if(1==data[j].Y) { error+=data[j].WEIGHT;
} } else if(data[j].X[feature]>threshold) { if(-1==data[j].Y) {
error+=data[j].WEIGHT; } } } if(error>0.5) { error=1-error; direction=1; }
else { direction=0; } if(error<min_error min_error="error;" i_thre="i;"
error="0;" double="" alpha="0;//该决策器的表决权" alpha="calAlpha(min_error);"
decision-="">threshold=i_thre; decision->weight=alpha;
decision->direction=direction; cout<<"阈值:"<<i_thre<<endl;
cout<<"权重："<<alpha<<endl; } int Adaboost::vote(int data[],int feature,Decision
*&decision,int LNum) { Decision *p; double h=0; p=decision; for(int j=0;j<lnum
j="" h="" calh="" data="" feature="" p="" p-="">weight; p=p->next; } if(h>0) {
h=1; } else { h=-1; } return h; } bool Adaboost::CheckEnd(Data data[],int
num,int feature,Decision *&decision,int LNum) { Decision *p; double *h=new
double[num]; //给h赋值，记录投票结果 for(int i=0;i<num;i++) { h[i]=0; p=decision;
for(int j=0;j<lnum j="" h="" i="" calh="" data="" i="" x="" feature="" p=""
p-="">weight; p=p->next; } if(h[i]>0) { h[i]=1; } else { h[i]=-1; } } //判断准确性
for(int i=0;i<num i="" if="" data="" i="" y="" h="" i="" delete="" h=""
return="" false="" delete="" h="" return="" true="" void=""
adaboost::destroylist="" decision="" l="" decision="" pre="L,*p=L-">next;
while(p!=NULL) { free(pre); pre=p; p=pre->next; } free(pre); } void
Adaboost::adaboost(Data data[],int num,int feature) { Decision
*decision;//定义决策器 Decision *Head;//定义决策器的头结点 int LinNum=0;//记录决策器的数量
CreateDecision(decision); Head=decision; LinNum++;
ThreAndWeight(data,num,feature,decision);
while(!CheckEnd(data,num,feature,Head,LinNum)) {
UpdateDataWeight(data,num,feature,decision); InsertDecision(decision);
ThreAndWeight(data,num,feature,decision); LinNum++; } cout<<LinNum<<endl;
cout<<"下面开始测试："<<endl; for(int i=0;i<num;i++) { cout<<"数据:"<<i<<":"<<endl; int
question[1]={i}; cout<<"结果："<<vote(question,feature,Head,LinNum)<<endl; }
//ThreAndWeight(data,num,feature,decision); DestroyList(Head); };
</num></lnum></lnum></min_error></num>  

  

  
  

  

