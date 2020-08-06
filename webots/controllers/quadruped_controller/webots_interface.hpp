 /**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 四足机器人 仿真环境或硬件接口 头文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#ifndef _WEBOTSINTERFACE_H_
#define _WEBOTSINTERFACE_H_

//-----------------------------------------------------------macro
#define PI           (3.141892654f)
#define TIME_STEP    (2)             //2ms
#define dt           (0.002)         //2ms = 0.002s
#define rad2deg      (57.2957795056f)//(180.0/PI)
#define deg2rad      ( 0.0174532925f)//(PI/180.0)
#define MAX_TORQUE   (1000)
//-----------------------------------------------------------typedef
/*
电机名称集合
*/
typedef enum
{
  FR0   = 0x00,
  FR1   = 0x01,
  FR2   = 0x02,
  
  FL0   = 0x03,
  FL1   = 0x04,
  FL2   = 0x05,
  
  BR0   = 0x06,
  BR1   = 0x07,
  BR2   = 0x08,
  
  BL0   = 0x09,
  BL1   = 0x0A,
  BL2   = 0x0B,
}motorNameTypeDef;


/* 
1，陀螺仪数据定义,为了方便调试采用角度制，注意，采用角度制
2，webots IMU模块采用RPY角度制，定系旋转，矩阵左乘，即：
       Rot=RotY(yaw)*RotZ(pitch)*RotX(roll);
3，eulerAngleTypeDef结构体描述了数学模型中的RPY，定系旋转，矩阵左乘，即：
       Rot=RotZ(yaw)*RotY(pitch)*RotX(roll);
4,由于webots默认坐标系和数学模型世界坐标系定义不同，因此二者RPY定义不同，对应关系如下：

==============================
*   数学模型   *   webots    *
------------------------------
*   Z(yaw)    *   Y(yaw)    *
*   Y(pitch)  *   Z(pitch)  *
*   X(roll)   *   X(roll)   *
==============================
*/
typedef struct
{
  float roll;       //横滚，x轴
  float pitch;      //俯仰，z轴
  float yaw;        //偏航，y轴
}eulerAngleTypeDef;

typedef struct
{
  float data[3];
}Vec3;
typedef struct
{
  float data[4];
}Vec4;
//-----------------------------------------------------------extern 
extern void              webots_device_init                                                                ();
extern void              set_motor_torque                          (motorNameTypeDef motorName, float torque);
extern void              set_motor_position                      (motorNameTypeDef motorName, float position);
extern double            get_motor_angle                                         (motorNameTypeDef motorName);
extern void              all_motor_stop                                                                    ();
extern Vec3              get_IMU_Angle                                                                     ();
extern Vec3              get_ACC                                                                           ();
extern Vec3              get_GYRO                                                                          ();
extern void              get_motor_angle_velocity (motorNameTypeDef motorName, float* angle, float* velocity);
#endif

