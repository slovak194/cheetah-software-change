/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 四足机器人 主 文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#include "stdio.h"
#include "assert.h" 
#include "thread"
#include "iostream"

#include <webots/Robot.hpp>
#include "webots_interface.hpp"

#include "webots2controller_t.hpp"
#include "controller2webots_t.hpp"
#include <lcm/lcm-cpp.hpp>

#include "quadruped_controller.hpp"

using namespace webots;
Robot *robot;
int iter_webots_loop = 0;

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  //device init
  webots_device_init();
  //lcm init
  lcm::LCM webots2controllerLCM("udpm://239.255.76.67:7667?ttl=255");
  lcm::LCM controller2webotsLCM("udpm://239.255.76.67:7667?ttl=255");
  webots2controller_t w2c;
  Handler handlerObject;
  handlerObject.zero();
  controller2webotsLCM.subscribe("controller2Webots", &Handler::handleController2WebotsLCM, &handlerObject);
  std::thread t1 = std::thread([&]() { for(;;) controller2webotsLCM.handle(); });

  while (robot->step(TIME_STEP) != -1) {

    //构造数据并发送
    Vec3 euler = get_IMU_Angle();
    Vec3 gyro = get_GYRO();
    Vec3 acc = get_ACC();
    for(int i=0;i<3;i++)
    {
      w2c.rpy[i] = euler.data[i];
      w2c.gyro[i] = gyro.data[i];
      w2c.acc[i] = acc.data[i];
    }
    for(LegTypeDef leg = FR; leg<=BL; leg = (LegTypeDef)(leg+1))
    {
      float q_abad,q_hip,q_knee,qd_abad,qd_hip,qd_knee;
      get_motor_angle_velocity((motorNameTypeDef)(leg*3 + 0), &q_abad, &qd_abad);
      get_motor_angle_velocity((motorNameTypeDef)(leg*3 + 1), &q_hip, &qd_hip);
      get_motor_angle_velocity((motorNameTypeDef)(leg*3 + 2), &q_knee, &qd_knee);
      w2c.q_abad[leg] = q_abad;
      w2c.q_hip[leg] = q_hip;
      w2c.q_knee[leg] = q_knee;
      w2c.qd_abad[leg] = qd_abad;
      w2c.qd_hip[leg] = qd_hip;
      w2c.qd_knee[leg] = qd_knee;
    }
    webots2controllerLCM.publish("webots2Controller",&w2c);
    //计算扭矩并执行
    controller2webots_t& c2w = handlerObject.c2w;
    for(LegTypeDef leg = FR; leg<=BL; leg = (LegTypeDef)(leg+1))
    {
      //计算力矩 = kp*(qdes - q) + kd*(qddes - qd) + ff;
      float torque_abad = c2w.kp_abad[leg]*(c2w.q_des_abad[leg] - w2c.q_abad[leg]) + 
                          c2w.kd_abad[leg]*(c2w.qd_des_abad[leg] - w2c.qd_abad[leg]) + 
                          c2w.tau_abad_ff[leg];
      float torque_hip  = c2w.kp_hip[leg]*(c2w.q_des_hip[leg] - w2c.q_hip[leg]) + 
                          c2w.kd_hip[leg]*(c2w.qd_des_hip[leg] - w2c.qd_hip[leg]) + 
                          c2w.tau_hip_ff[leg];
      float torque_knee = c2w.kp_knee[leg]*(c2w.q_des_knee[leg] - w2c.q_knee[leg]) + 
                          c2w.kd_knee[leg]*(c2w.qd_des_knee[leg] - w2c.qd_knee[leg]) + 
                          c2w.tau_knee_ff[leg];
      //set torque
      set_motor_torque((motorNameTypeDef)(leg*3+0), torque_abad);
      set_motor_torque((motorNameTypeDef)(leg*3+1), torque_hip);
      set_motor_torque((motorNameTypeDef)(leg*3+2), torque_knee);

      if(leg == FR)
      {
       // printf("%f %f %f \r\n",torque_abad, torque_hip, torque_knee);
      }
    }
   // printf("we done a circle!\r\n");
  //iter num
  iter_webots_loop++;
   if(iter_webots_loop == 500)
   {
     printf("iter_webots_loop && iter_ctrl_loop: %d, %d\r\n",iter_webots_loop,handlerObject.iter);
     iter_webots_loop = 0;
     handlerObject.iter = 0;
   }
  }

  delete robot;
  t1.join();
  return 0;
}
