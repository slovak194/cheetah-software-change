## 魔改 Cheetah-Software
### 2020.07.12
搭建webots模型,从solidworks导入外观模型
导出abad\hip\knee\body的惯性张量
### 2020.07.14
设置webots中关节范围,后面考虑修改,从程序修改?加入排斥力?
新建webotsbridge.cpp,通过lcm与webots模型通信
### 2020.07.20
将自己的机器人模型导入算法的仿真中,删掉cheetah3模型
修改minicheetah.h
修改仿真中关于电机仿真的程序,因为不知道电机内阻,因此去掉电压导致的电机扭矩限制
### 2020.07.25
cmpclocomotion.cpp中,高速移动时的一些trick,删掉了
### 2020.07.28
世界坐标系下期望位置,x方向由期望速度积分,y方向=状态估计+dt*期望速度.
减小abad关节刚度
todo:如何最优调参?
### 2020.08.01
将部分eigen变量由动态改成静态,似乎加快了mpc计算效率,但是编译是真的慢
删掉了部分没有用到的参数
### 2020.08.02
由于用不到eathercat,删掉了SOEM
去掉了yaml中没用到的参数
todo:出于安全考虑,大改recover_stand,形成类似stand_up,或者考虑新建一个stand_up状态
todo:将所有魔法数字调整到yaml中
