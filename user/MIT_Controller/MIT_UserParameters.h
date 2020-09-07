#ifndef PROJECT_MITUSERPARAMETERS_H
#define PROJECT_MITUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class MIT_UserParameters : public ControlParameters {
public:
  MIT_UserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(use_wbc),
        INIT_PARAMETER(floating_base_weight),
        INIT_PARAMETER(stride_max),
        INIT_PARAMETER(cmpc_mu),
        INIT_PARAMETER(cmpc_fmax),
        INIT_PARAMETER(foot_hight),
        INIT_PARAMETER(foot_location_k),
        INIT_PARAMETER(sit_down_height),
        INIT_PARAMETER(stand_up_height),
        INIT_PARAMETER(sit_down_time),
        INIT_PARAMETER(stand_up_time),
        INIT_PARAMETER(Swing_step_offset),
        INIT_PARAMETER(Kp_stand),
        INIT_PARAMETER(Kd_stand),
        INIT_PARAMETER(Kp_body),
        INIT_PARAMETER(Kd_body),
        INIT_PARAMETER(Kp_ori),
        INIT_PARAMETER(Kd_ori),
        INIT_PARAMETER(Kp_foot),
        INIT_PARAMETER(Kd_foot),
        INIT_PARAMETER(Kp_joint),
        INIT_PARAMETER(Kd_joint),
        INIT_PARAMETER(mpc_kp_foot),
        INIT_PARAMETER(mpc_kd_foot),
        INIT_PARAMETER(mpc_kp_stance_foot),
        INIT_PARAMETER(mpc_kd_stance_foot)

  {}
  DECLARE_PARAMETER(double, use_wbc);
  DECLARE_PARAMETER(double, floating_base_weight);
  DECLARE_PARAMETER(double, stride_max);
  DECLARE_PARAMETER(double, cmpc_mu);
  DECLARE_PARAMETER(double, cmpc_fmax);
  DECLARE_PARAMETER(double, foot_hight);
  DECLARE_PARAMETER(double, foot_location_k);
  DECLARE_PARAMETER(double, sit_down_height);
  DECLARE_PARAMETER(double, stand_up_height);
  DECLARE_PARAMETER(double, sit_down_time);
  DECLARE_PARAMETER(double, stand_up_time);
  DECLARE_PARAMETER(Vec3<double>, Swing_step_offset);
  
  DECLARE_PARAMETER(Vec3<double>, Kp_stand);
  DECLARE_PARAMETER(Vec3<double>, Kd_stand);

  DECLARE_PARAMETER(Vec3<double>, Kp_body);
  DECLARE_PARAMETER(Vec3<double>, Kd_body);

  DECLARE_PARAMETER(Vec3<double>, Kp_ori);
  DECLARE_PARAMETER(Vec3<double>, Kd_ori);

  DECLARE_PARAMETER(Vec3<double>, Kp_foot);
  DECLARE_PARAMETER(Vec3<double>, Kd_foot);

  DECLARE_PARAMETER(Vec3<double>, Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint);

  DECLARE_PARAMETER(Vec3<double>, mpc_kp_foot);
  DECLARE_PARAMETER(Vec3<double>, mpc_kd_foot);

  DECLARE_PARAMETER(Vec3<double>, mpc_kp_stance_foot);
  DECLARE_PARAMETER(Vec3<double>, mpc_kd_stance_foot);
};

#endif //PROJECT_MITUSERPARAMETERS_H
