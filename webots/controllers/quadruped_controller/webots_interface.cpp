/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 四足机器人 仿真环境 接口文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#include <stdio.h>
#include <vector>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>

#include "webots_interface.hpp"

using namespace webots;
//-----------------------------------------------------------device
/*电机*/
Motor *motor[12];

/*电机编码器*/
PositionSensor *pos_sensor[12];

/*惯导系统*/
InertialUnit *IMU;
Accelerometer *ACC;
Gyro* GYRO;

/*
函数功能：初始化devices
*/
void webots_device_init()
{
  extern Robot *robot;
  // create the Robot instance.
  robot = new Robot();
  //get device
  motor[FR0]          = robot->getMotor("fr abad rotational motor");
  motor[FR1]          = robot->getMotor("fr hip rotational motor");
  motor[FR2]          = robot->getMotor("fr knee rotational motor");
  
  motor[FL0]          = robot->getMotor("fl abad rotational motor");
  motor[FL1]          = robot->getMotor("fl hip rotational motor");
  motor[FL2]          = robot->getMotor("fl knee rotational motor");
  
  motor[BR0]          = robot->getMotor("br abad rotational motor");
  motor[BR1]          = robot->getMotor("br hip rotational motor");
  motor[BR2]          = robot->getMotor("br knee rotational motor");
  
  motor[BL0]          = robot->getMotor("bl abad rotational motor");
  motor[BL1]          = robot->getMotor("bl hip rotational motor");
  motor[BL2]          = robot->getMotor("bl knee rotational motor");
  
  pos_sensor[FR0]     = robot->getPositionSensor("fr abad position sensor");
  pos_sensor[FR1]     = robot->getPositionSensor("fr hip position sensor");
  pos_sensor[FR2]     = robot->getPositionSensor("fr knee position sensor");
  
  pos_sensor[FL0]     = robot->getPositionSensor("fl abad position sensor");
  pos_sensor[FL1]     = robot->getPositionSensor("fl hip position sensor");
  pos_sensor[FL2]     = robot->getPositionSensor("fl knee position sensor");
  
  pos_sensor[BR0]     = robot->getPositionSensor("br abad position sensor");
  pos_sensor[BR1]     = robot->getPositionSensor("br hip position sensor");
  pos_sensor[BR2]     = robot->getPositionSensor("br knee position sensor");
  
  pos_sensor[BL0]     = robot->getPositionSensor("bl abad position sensor");
  pos_sensor[BL1]     = robot->getPositionSensor("bl hip position sensor");
  pos_sensor[BL2]     = robot->getPositionSensor("bl knee position sensor");
  
  IMU                = robot->getInertialUnit("inertial unit");
  ACC                = robot->getAccelerometer("accelerometer");
  GYRO               = robot->getGyro("gyro");
  
  //enable
  for(motorNameTypeDef motor = FR0; motor <=BL2; motor=(motorNameTypeDef)(motor+1))
  {
    pos_sensor[motor]->enable(TIME_STEP);
  }
  IMU->enable(TIME_STEP);
  ACC->enable(TIME_STEP);
  GYRO->enable(TIME_STEP);
}
//-----------------------------------------------------------motor
/*
函数功能：设置电机扭矩
*/
void set_motor_torque(motorNameTypeDef motorName, float torque)
{
  if(torque >  MAX_TORQUE)torque =  MAX_TORQUE;
  if(torque < -MAX_TORQUE)torque = -MAX_TORQUE;

  motor[motorName]->setTorque((double)torque);
}
/*
函数功能：设置电机位置
*/
void set_motor_position(motorNameTypeDef motorName, float position)
{
  double offset[12]={PI,PI,0,0,0,0,PI,PI,0,0,0,0};
  position += offset[motorName];
  motor[motorName]->setPosition((double)position);
}
//-----------------------------------------------------------sensor
/*
函数功能：获取电机角度,角速度,单位rad
注意:此函数必须每控制周期调用一次.否则速度计算会出问题
*/
void get_motor_angle_velocity(motorNameTypeDef motorName, float* angle, float* velocity)
{
  //angle
  float offset[12]={-PI,-PI,0,0,0,0,-PI,-PI,0,0,0,0};
  *angle = (float)(pos_sensor[motorName]->getValue());
  *angle += offset[motorName];
  //velocity
  float filter = 0.3f;
  static float pre_angle[12] = {-PI,-PI,0,0,0,0,-PI,-PI,0,0,0,0};
  static float pre_velocity[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
  *velocity = ((*angle)-pre_angle[motorName])/dt;
  *velocity = *velocity*filter + pre_velocity[motorName]*(1-filter);
  //backup
  pre_angle[motorName] = *angle;
  pre_velocity[motorName] = *velocity;
}

/*
函数功能：读取IMU数据
*/

Vec3 get_IMU_Angle()
{
  const double* data = IMU->getRollPitchYaw();
  
  Vec3 eulerAngle;
  eulerAngle.data[0]  = (float)data[0];  //roll
  eulerAngle.data[1]  = -(float)data[1]; //pitch
  eulerAngle.data[2]  = (float)data[2];  //yaw
  
  return eulerAngle;
}
/*
读取加速度
*/
Vec3 get_ACC()
{
  Vec3 acc;
  const double* a = ACC->getValues();
  acc.data[0] = a[0];
  acc.data[1] = a[1];
  acc.data[2] = a[2];
  return acc;
}
/*
读取角速度
*/
Vec3 get_GYRO()
{
  Vec3 gyro;
  const double* w = GYRO->getValues();
  gyro.data[0] = w[0];
  gyro.data[1] = w[1];
  gyro.data[2] = w[2];
  return gyro;
}
/*
函数功能：所有电机抱闸停止运动
*/
void all_motor_stop()
{
  for(motorNameTypeDef motor = FR0; motor <=BL2; motor=(motorNameTypeDef)(motor+1))
  {
    float angle,velocity;
    get_motor_angle_velocity(motor, &angle, &velocity);
    set_motor_position(motor, angle);
  }
}


