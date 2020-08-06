 /**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 四足机器人 控制器 头文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#ifndef _QUADRUPED_CONTROLLER_H_
#define _QUADRUPED_CONTROLLER_H_

#include "stdio.h"
#include "iostream"

#include "controller2webots_t.hpp"
#include <lcm/lcm-cpp.hpp>
//-----------------------------------------------------------macro
// some macro
//-----------------------------------------------------------typedef
/*
腿的序号
*/
typedef enum
{
  FR = 0x00,
  FL = 0x01,
  BR = 0x02,
  BL = 0x03
}LegTypeDef;
/*
lcm 信息处理类
*/
class Handler 
{
  public:
  controller2webots_t c2w;
  int iter = 0;
    ~Handler() {}
/*!
 * 处理从controller传递来的数据
 */
  void handleController2WebotsLCM(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& chan,
                                  const controller2webots_t* msg) {
    (void)rbuf;
    (void)chan;
    memcpy(&c2w, msg, sizeof(controller2webots_t));
    iter++;
    //printf("get a c2w data\r\n");
  }
  void zero()
  {
    for(int i=0;i<4;i++)
    {
        c2w.q_des_abad[i]=0;
        c2w.q_des_hip[i]=0;
        c2w.q_des_knee[i]=0;
        c2w.qd_des_abad[i]=0;
        c2w.qd_des_hip[i]=0;
        c2w.qd_des_knee[i]=0;
        c2w.kp_abad[i]=0;
        c2w.kp_hip[i]=0;
        c2w.kp_knee[i]=0;
        c2w.kd_abad[i]=0;
        c2w.kd_hip[i]=0;
        c2w.kd_knee[i]=0;
        c2w.tau_abad_ff[i]=0;
        c2w.tau_hip_ff[i]=0;
        c2w.tau_knee_ff[i]=0;
        c2w.flags[i]=1;
    }
  }
};

#endif

