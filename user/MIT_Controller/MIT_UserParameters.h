#ifndef PROJECT_MITUSERPARAMETERS_H
#define PROJECT_MITUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class MIT_UserParameters : public ControlParameters {
public:
  MIT_UserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(stride_max),
        INIT_PARAMETER(cmpc_mu),
        INIT_PARAMETER(cmpc_fmax),
        INIT_PARAMETER(foot_hight),
        INIT_PARAMETER(foot_location_k),
        INIT_PARAMETER(sit_down_height),
        INIT_PARAMETER(stand_up_height),
        INIT_PARAMETER(sit_down_time),
        INIT_PARAMETER(cmpc_gait),
        INIT_PARAMETER(use_wbc),
        INIT_PARAMETER(Swing_step_offset),
        INIT_PARAMETER(Kp_body),
        INIT_PARAMETER(Kd_body),
        INIT_PARAMETER(Kp_ori),
        INIT_PARAMETER(Kd_ori),
        INIT_PARAMETER(Kp_foot),
        INIT_PARAMETER(Kd_foot),
        INIT_PARAMETER(Kp_joint),
        INIT_PARAMETER(Kd_joint),

        INIT_PARAMETER(gait_type),
        INIT_PARAMETER(gait_period_time),
        INIT_PARAMETER(gait_switching_phase),
        INIT_PARAMETER(gait_override),
        INIT_PARAMETER(gait_max_leg_angle),
        INIT_PARAMETER(gait_max_stance_time),
        INIT_PARAMETER(gait_min_stance_time)

  {}
  DECLARE_PARAMETER(double, stride_max);
  DECLARE_PARAMETER(double, cmpc_mu);
  DECLARE_PARAMETER(double, cmpc_fmax);
  DECLARE_PARAMETER(double, foot_hight);
  DECLARE_PARAMETER(double, foot_location_k);
  DECLARE_PARAMETER(double, sit_down_height);
  DECLARE_PARAMETER(double, stand_up_height);
  DECLARE_PARAMETER(double, sit_down_time);
  DECLARE_PARAMETER(double, cmpc_gait);
  DECLARE_PARAMETER(double, use_wbc);
  DECLARE_PARAMETER(Vec3<double>, Swing_step_offset);

  DECLARE_PARAMETER(Vec3<double>, Kp_body);
  DECLARE_PARAMETER(Vec3<double>, Kd_body);

  DECLARE_PARAMETER(Vec3<double>, Kp_ori);
  DECLARE_PARAMETER(Vec3<double>, Kd_ori);

  DECLARE_PARAMETER(Vec3<double>, Kp_foot);
  DECLARE_PARAMETER(Vec3<double>, Kd_foot);

  DECLARE_PARAMETER(Vec3<double>, Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint);

  // Gait Scheduler
  DECLARE_PARAMETER(double, gait_type);
  DECLARE_PARAMETER(double, gait_period_time);
  DECLARE_PARAMETER(double, gait_switching_phase);
  DECLARE_PARAMETER(double, gait_override);
  DECLARE_PARAMETER(double, gait_max_leg_angle);
  DECLARE_PARAMETER(double, gait_max_stance_time);
  DECLARE_PARAMETER(double, gait_min_stance_time);

};

#endif //PROJECT_MITUSERPARAMETERS_H
