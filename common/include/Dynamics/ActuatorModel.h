/*! @file ActuatorModel.h
 *  @brief Model of actuator
 *  Includes friction, max torque, and motor torque speed curve.
 *
 *  The getTorque is used for torque at the joint, not torque at the motor.
 *  The provided frictions are for torques at the joint, not torque at the motor
 *  The R/KT are for the motor
 * getTorque函数用于关节处的扭矩,而不是电机处的扭矩.
 * friction是用于关节处的扭矩,而不是电机的扭矩
 * R/KT是电机的参数
 */

#ifndef PROJECT_ACTUATORMODEL_H
#define PROJECT_ACTUATORMODEL_H

#include "Utilities/utilities.h"

/*!
 * A model of an actuator containing friction and electrical effects
 * 一个包含摩擦和电气参数的执行器模型
 */
template <typename T>
class ActuatorModel {
 public:

  /*!
   * Construct a new actuator model with the given parameters
   * @param gearRatio : Gear reduction  减速比
   * @param motorKT : Value of KT (torque constant) for the motor转矩系数.单位  Nm/A
   * @param motorR : Motor resistance 电机电阻 
   * @param batteryV : Battery voltage 电池电压
   * @param damping : Actuator damping (at the joint, Nm/(rad/sec))  阻尼
   * @param dryFriction : Actuator dry friction (at the joint, Nm)   干摩擦
   * @param tauMax : Maximum torque output of the actuator  执行器最大电流
   */
  ActuatorModel(T gearRatio, T motorKT, T motorR, T batteryV, T damping,
                T dryFriction, T tauMax)
      : _gr(gearRatio),
        _kt(motorKT),
        _R(motorR),
        _V(batteryV),
        _damping(damping),
        _dryFriction(dryFriction),
        _tauMax(tauMax) {}

  ActuatorModel() {}

  // compute

  /*!
   * Compute actual actuator torque, given desired torque and speed.
   * takes into account friction (dry and damping), voltage limits, and torque
   * limits
   * 这个函数好像只是在仿真环境下使用了,没有在robots环境下使用
   * @param tauDes : desired torque
   * @param qd : current actuator velocity (at the joint)
   * @return actual produced torque
   */
  T getTorque(T tauDes, T qd) {

    //这里,我去掉了电气因素对最大扭矩的限制,因为智擎未提供电机相关参数
    /*
    // compute motor torque
    T tauDesMotor = tauDes / _gr;        // motor torque
    T iDes = tauDesMotor / (_kt * 1.5);  // i = tau / KT    //为什么x1.5?大概是换电机了?
    // T bemf =  qd * _gr * _kt * 1.732;     // back emf
    T bemf = qd * _gr * _kt * 2.;       // back emf
    T vDes = iDes * _R + bemf;          // v = I*R + emf
    T vActual = coerce(vDes, -_V, _V);  // limit to battery voltage
    T tauActMotor =
        1.5 * _kt * (vActual - bemf) / _R;  // tau = Kt * I = Kt * V / R
    T tauAct = _gr * coerce(tauActMotor, -_tauMax, _tauMax);
    */
    T tauAct = coerce(tauDes , -_tauMax*_gr, _tauMax*_gr);

    // add damping and dry friction
    if (_frictionEnabled)
      tauAct = tauAct - _damping * qd - _dryFriction * sgn(qd);

    return tauAct;
  }

  /*!
   * Control friction effects
   * @param enabled : enable/disable both dry and damping friction terms
   */
  void setFriction(bool enabled) { _frictionEnabled = enabled; }

 private:
  T _gr, _kt, _R, _V, _damping, _dryFriction, _tauMax;
  bool _frictionEnabled = true;
};

#endif  // PROJECT_ACTUATORMODEL_H
