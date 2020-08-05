#ifndef CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

#include <Controllers/FootSwingTrajectory.h>
#include <FSM_States/ControlFSMData.h>
#include "cppTypes.h"
#include "Gait.h"

#include <cstdio>

using Eigen::Array4f;
using Eigen::Array4i;


template<typename T>
struct CMPC_Result {
  LegControllerCommand<T> commands[4];
  Vec4<T> contactPhase;
};

class ConvexMPCLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters);
  void initialize();

  template<typename T>
  void run(ControlFSMData<T>& data);

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;
  Vec4<float> swing_state;


private:
  void _SetupCommand(ControlFSMData<float> & data);

  float _yaw_turn_rate;
  float _yaw_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  float _body_height;

  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data);
  void solveDenseMPC(int *mpcTable, ControlFSMData<float> &data);
  int iterationsBetweenMPC;
  int horizonLength;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait trotting, bounding, pronking, galloping, standing, trotRunning, walking, walking2, pacing;
  MixedFrequncyGait random, random2;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  int current_gait;
  int gaitNumber;
  float stand_traj[6];
  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  Vec3<float> pFoot[4];
  CMPC_Result<float> result;
  float trajAll[12*36];

  MIT_UserParameters* _parameters = nullptr;

  vectorAligned<Vec12<double>> _sparseTrajectory;

};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
