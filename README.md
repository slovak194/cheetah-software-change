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
# 2020.08.03
给机器人起名字cheetah,程序中做了相应更改

删掉了jposiniter,感觉初次调试用这个很危险,后期加入到FSM中

精简FSM

出于安全考虑,大改recover_stand,形成类似stand_up,或者考虑新建一个stand_up状态

# 2020.08.04
删去未使用task

去掉control_mode,设计操作逻辑流程,改用手柄完全操作FSM

解决手柄左边4个按键无效问题

todo:设计策略,侧向运动时腿与腿不打架

删去tiboatd相关内容

四元数统一格式xyzw

删除 common/FootstepPlanner

添加梯形加减速曲线T_Curve,原来的B样条曲线只是规划路径,在路径上匀速,无法实现启动后加速与到位前减速的效果.

为abad link添加接触点
# 2020.08.05
加入standup与sitdown加减速过程

解决使用手柄调度FSM的一个bug

todo:修改设计安全检查策略
# 2020.08.06
实现了一个足底接触判断,但是效果并不好,容易发散,于是删了
# 2020.08.08
新增参数kp_stand与kd_stand,在stand_up环节与sit_down环节作为足底笛卡尔空间使用

发现论文中Q1与Q2参数,Q2原本设为0.1,改为1000后效果非常好,但是其他步态效果不好

FSM中加入balance_stand,并设计相关逻辑

使用手柄切换步态,删去不用以及不稳定的步态

禁止了行走过程中切换步态

todo:调试时,Q2根据不同步态改变

# 2020.08.16

优化MPC计算效率，使用对角矩阵，使用静态矩阵，指针赋值等。

todo:joint PD是否需要摆动相与支持项分离？
# 2020.08.18
测试mkl,效果不明显
# 2020.08.20
controller使用独立gamepad,不使用Qt库
# 2020.08.31
提高eigen矩阵乘法效率
# 2020.09.01
解决了机器人启动踏步瞬间抽搐问题，iterationCounter应该在最后自增
todo:重新加入use_wbc用于调试




