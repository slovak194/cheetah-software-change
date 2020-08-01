#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>
#include "SolverMPC.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../../../../common/FootstepPlanner/GraphSearch.h"

#include "Gait.h"

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(HORIZON_LENGTH),
  dt(_dt),
  //trotting(10, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
  //trotting(24, Vec4<int>(0,12,12,0), Vec4<int>(12,12,12,12),"Trotting"),
  trotting(18, Vec4<int>(0,9,9,0), Vec4<int>(9,9,9,9),"Trotting"),
  //trotting(12, Vec4<int>(0,6,6,0), Vec4<int>(6,6,6,6),"Trotting"),
  //trotting(30, Vec4<int>(0,15,15,0), Vec4<int>(15,15,15,15),"Trotting"),
  //trotting(120, Vec4<int>(0,60,60,0), Vec4<int>(110,110,110,110),"Trotting"),
  bounding(10, Vec4<int>(5,5,0,0),Vec4<int>(5,5,5,5),"Bounding"),
  //bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
  pronking(10, Vec4<int>(0,0,0,0),Vec4<int>(5,5,5,5),"Pronking"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
  galloping(horizonLength, Vec4<int>(0,4,14,17),Vec4<int>(8,8,8,8),"Galloping"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(18,18,18,18),"Standing"),
  //trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running"),
  trotRunning(horizonLength, Vec4<int>(0,9,9,0),Vec4<int>(7,7,7,7),"Trot Running"),
  walking(horizonLength, Vec4<int>(0,6,10,16), Vec4<int>(9,9,9,9), "Walking"),
  walking2(horizonLength, Vec4<int>(0,9,9,0), Vec4<int>(14,14,14,14), "Walking2"),
  pacing(horizonLength, Vec4<int>(9,0,9,0),Vec4<int>(9,9,9,9),"Pacing"),
  random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
  random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, _parameters->cmpc_mu, _parameters->cmpc_fmax);

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

   pBody_des.setZero();
   vBody_des.setZero();
   aBody_des.setZero();

   _body_height = parameters->stand_up_height; 
}

void ConvexMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){

  float x_vel_cmd, y_vel_cmd,_body_height_cmd;
  float filter(0.002);              //注意这个滤波器,决定了行走的加速度,很重要
  float filter2(0.006);              //注意这个滤波器,很重要
  //cmpc_gait  _yaw_turn_rate   x_vel_cmd   y_vel_cmd  _body_height
  //vel
  _yaw_turn_rate = 0.5*(data._gamepadCommand->leftTriggerAnalog - data._gamepadCommand->rightTriggerAnalog);
  x_vel_cmd =  data._gamepadCommand->leftStickAnalog[1]*0.8;
  y_vel_cmd = -data._gamepadCommand->rightStickAnalog[0]*0.35;
  //filter
  _x_vel_des = _x_vel_des*(1-filter) + x_vel_cmd*filter;
  _y_vel_des = _y_vel_des*(1-filter) + y_vel_cmd*filter;
  //yaw
  _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  //height
  _body_height_cmd = data.userParameters->stand_up_height; 
  if(data._gamepadCommand->a) _body_height_cmd *= 0.7;  //下蹲指令
 
  _body_height = _body_height*(1-filter2) + _body_height_cmd*filter2;
}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data) {
  // 状态估计
  auto& seResult = data._stateEstimator->getResult();
  // 初始化一些状态
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for(int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(_parameters->foot_hight);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }
  // 从手柄读取命令
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;

 // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = _body_height;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }
  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &pronking;
  else if(gaitNumber == 3)
    gait = &random;
  else if(gaitNumber == 4)
    gait = &standing;
  else if(gaitNumber == 5)
    gait = &trotRunning;
  else if(gaitNumber == 6)
    gait = &random2;
  else if(gaitNumber == 7)
    gait = &random2;
  else if(gaitNumber == 8)
    gait = &pacing;
  current_gait = gaitNumber;

  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  iterationCounter++;

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = seResult.rBody.transpose() * v_des_robot;

  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);
  }

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // foot placement
  for(int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  float x_side_sign[4] = {1, 1, -1, -1};
  float y_side_sign[4] = {-1, 1, -1, 1};
  for(int i = 0; i < 4; i++)
  {
    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    footSwingTrajectories[i].setHeight(_parameters->foot_hight);
    Vec3<float> abad_offset(0, y_side_sign[i] * data._quadruped->_abadLinkLength, 0);
    Vec3<float> user_offset(x_side_sign[i]*_parameters->Swing_step_offset[0], 
                            y_side_sign[i]*_parameters->Swing_step_offset[1],
                            0);

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + abad_offset + user_offset);

    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected = 
      coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2.0f) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
          + des_vel * swingTimeRemaining[i]);

    // 计算落脚点
    float pfx_rel = seResult.vWorld[0] * 0.5 * stance_time +
      _parameters->foot_location_k*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*sqrt(fabs(seResult.position[2]/9.81f))) * (seResult.vWorld[1]*_yaw_turn_rate);

    float pfy_rel = seResult.vWorld[1] * 0.5 * stance_time +
      _parameters->foot_location_k*(seResult.vWorld[1]-v_des_world[1]) +
      (0.5f*sqrt(fabs(seResult.position[2]/9.81f))) * (-seResult.vWorld[0]*_yaw_turn_rate);

    // 限幅
    pfx_rel = fminf(fmaxf(pfx_rel, -_parameters->stride_max), _parameters->stride_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -_parameters->stride_max), _parameters->stride_max);
    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    Pf[2] =  0;
    footSwingTrajectories[i].setFinalPosition(Pf);
  }
  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();

  updateMPCIfNeeded(mpcTable, data);

  for(int foot = 0; foot < 4; foot++)
  {
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(contactStates);

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.; 
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data) {

  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);
    Vec3<float> v_des_world = seResult.rBody.transpose() * v_des_robot;

    if(current_gait == 4)
    {
      float trajInitial[12] = {
        0,
        0,
        (float)stand_traj[5],
        (float)stand_traj[0],
        (float)stand_traj[1],
        (float)_body_height,
        0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    }
    else
    {
      float trajInitial[12] = {
        0.0,                                      // 0
        0.0,                                      // 1
        _yaw_des,                                 // 2
        world_position_desired[0],                // 3
        world_position_desired[1],                // 4
        (float)_body_height,                      // 5
        0,                                        // 6
        0,                                        // 7
        _yaw_turn_rate,                           // 8
        v_des_world[0],                           // 9
        v_des_world[1],                           // 10
        0};                                       // 11

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    Timer solveTimer;

    solveDenseMPC(mpcTable, data);
    //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());  //查看计算时间
  }

}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult();

   float Q[12] = {0.25, 0.25, 10,    //角度
                  2, 2, 50,          //位置
                  0, 0, 0.3,         //角速度
                  0.2, 0.2, 0.1};    //速度

  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-7;         // 力的权重
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);

  Timer t2;
  update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);
  printf("MPC Solve time %f ms\n", t2.getMs());

  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg*3 + axis);

    //printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}