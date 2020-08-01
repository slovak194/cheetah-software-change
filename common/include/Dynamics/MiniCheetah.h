/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  //机器人基本尺寸参数
  cheetah._bodyMass         = 6.89;
  cheetah._bodyLength       = 0.275*2;
  cheetah._bodyWidth        = 0.0625*2;   //左右电机轴距
  cheetah._bodyHeight       = 0.047*2;
  cheetah._abadGearRatio    = 6;
  cheetah._hipGearRatio     = 6;
  cheetah._kneeGearRatio    = 6;
  cheetah._abadLinkLength   = 0.088;      //第一连杆长度
  cheetah._hipLinkLength    = 0.270;      //第二连杆长度
  cheetah._kneeLinkY_offset = 0;          //足底碰撞模型的偏移量，不需要
  cheetah._kneeLinkLength   = 0.270;      //第三连杆长度
  cheetah._maxLegLength     = 0.465-0.01; //腿总长。abad总是垂直于hip和knee，因此该值取决于大小腿角度，1厘米安全距离

  //以下参数仅在simulator模式下使用，用于仿真器中模拟电机，我们用不到
  cheetah._motorTauMax      = 8.0f;       //3.f;
  cheetah._batteryV         = 36;         //24;
  cheetah._motorKT          = .05;        //this is flux linkage * pole pairs
  cheetah._motorR           = 0.173;
  cheetah._jointDamping     = .01;
  cheetah._jointDryFriction = .2;

  // rotor inertia if the rotor is oriented so it spins around the z-axis
  // 转子转动惯量
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 62, 0, 0, 0, 62, 0, 0, 0, 120;  // 单位 kg*(mm)^2
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;  // 单位 kg*m^2

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  // 三个杆的空间惯量
  // 注意，Inertia是质心处的惯性张量
  // com的坐标系为当前body-fix坐标系
  Mat3<T> abadRotationalInertia;
  //abad
  abadRotationalInertia << 475.8, -4.152, 0.0, -4.152, 780.4, 0.0, 0.0, 0.0, 563.3;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(-0.00401, -0.002058, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.707, abadCOM, abadRotationalInertia);
  //hip
  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 12828.12970, 172.41965, 701.27894, 172.41965, 12887.34375, -1392.96942, 701.27894, -1392.96942, 1408.71425;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0.00728, 0.01745, -0.05903);
  SpatialInertia<T> hipInertia(1.336, hipCOM, hipRotationalInertia);
  //knee
  Mat3<T> kneeRotationalInertia;
  kneeRotationalInertia << 3762.21189, -0.06977, 29.77648, -0.06977, 3784.14643, 0.05612, 29.77648, 0.05612, 56.01498;
  kneeRotationalInertia = kneeRotationalInertia * 1e-6;
  Vec3<T> kneeCOM(-0.00108, 0, -0.11839);
  SpatialInertia<T> kneeInertia(0.297, kneeCOM, kneeRotationalInertia);

  //转子的空间惯量
  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.098, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.098, rotorCOM, rotorRotationalInertiaY);

  //机身的空间惯量
  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 32742.5, 0, 0, 0, 151025, 0, 0, 0, 174847.4;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  cheetah._abadInertia = abadInertia;
  cheetah._hipInertia = hipInertia;
  cheetah._kneeInertia = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia = bodyInertia;

  // locations
  // 转子与关节的坐标
  cheetah._abadRotorLocation = Vec3<T>(0.195, 0.0625, 0);
  cheetah._abadLocation =
      Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._hipRotorLocation = Vec3<T>(0, 0, 0);
  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0.035, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H
