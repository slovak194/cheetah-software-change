## 魔改 Cheetah-Software
# 2020.07.12
搭建webots模型,从solidworks导入外观模型

导出abad\hip\knee\body的惯性张量
# 2020.07.14
设置webots中关节范围,后面考虑修改,从程序修改?加入排斥力?

新建webotsbridge.cpp,通过lcm与webots模型通信
# 2020.07.20
将自己的机器人模型导入算法的仿真中,删掉cheetah3模型

修改minicheetah.h

修改仿真中关于电机仿真的程序,因为不知道电机内阻,因此去掉电压导致的电机扭矩限制
# 2020.07.25
cmpclocomotion.cpp中,高速移动时的一些trick,删掉了
# 2020.07.28
世界坐标系下期望位置,x方向由期望速度积分,y方向=状态估计+dt*期望速度.

减小abad关节刚度

todo:如何最优调参?
# 2020.08.01
将部分eigen变量由动态改成静态,似乎加快了mpc计算效率,但是编译是真的慢

删掉了部分没有用到的参数
# 2020.08.02
由于用不到eathercat,删掉了SOEM

去掉了yaml中没用到的参数

todo:将所有魔法数字调整到yaml中
# 2020.08.02
给机器人起名字cheetah,程序中做了相应更改

删掉了jposiniter,感觉初次调试用这个很危险,后期加入到FSM中

精简FSM

出于安全考虑,recover_stand,改成简单的stand_up状态

todo:四元数统一格式

todo:完善foot速度位置状态估计,并根据foot位置与速度以及时间,做足底估计接触.可以考虑在webots下用接触传感器辅助调参
# 2020.08.04
删去未使用task

去掉control_mode,设计操作逻辑流程,改用手柄完全操作FSM

添加为abad link添加了接触点,防止passive状态下穿模

todo:统一笛卡尔坐标系下足底PD控制器参数








