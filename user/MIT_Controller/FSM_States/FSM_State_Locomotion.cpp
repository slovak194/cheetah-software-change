/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_Locomotion.h"
#include <Utilities/Timer.h>
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
//#include <rt/rt_interface_lcm.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
      30 / (1000. * _controlFSMData->controlParameters->controller_dt),
      _controlFSMData->userParameters);

  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF and footstep locations to 0s
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();
}

template <typename T>
void FSM_State_Locomotion<T>::onEnter() {

  printf("[FSM_State_Locomotion] onEnter...\n");
  cMPCOld->initialize();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Locomotion<T>::run() {
  // Call the locomotion control logic for this iteration
  LocomotionControlStep();
}
template<typename T>
bool FSM_State_Locomotion<T>::locomotionSafe() {
  auto& seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;
  // Safety parameters
  T maxPDes = 0.36;

  if(std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if(std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for(int leg = 0; leg < 4; leg++) {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if(p_leg[2] > 0) {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if(std::fabs(p_leg[1] > maxPDes)) {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    if(std::fabs(p_leg[0] > maxPDes)) {
      printf("Unsafe locomotion: leg %d's x-position is bad (%.3f m)\n", leg, p_leg[0]);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if(std::fabs(v_leg) > 6.) {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;

}


/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep() {
  //mpc计算
  cMPCOld->run<T>(*this->_data);
  //wbc期望数据填充
  _wbc_data->pBody_des = cMPCOld->pBody_des;
  _wbc_data->vBody_des = cMPCOld->vBody_des;
  _wbc_data->aBody_des = cMPCOld->aBody_des;

  _wbc_data->pBody_RPY_des = cMPCOld->pBody_RPY_des;
  _wbc_data->vBody_Ori_des = cMPCOld->vBody_Ori_des;
  
  for(size_t i(0); i<4; ++i){
    _wbc_data->pFoot_des[i] = cMPCOld->pFoot_des[i];
    _wbc_data->vFoot_des[i] = cMPCOld->vFoot_des[i];
    _wbc_data->aFoot_des[i] = cMPCOld->aFoot_des[i];
    _wbc_data->Fr_des[i] = cMPCOld->Fr_des[i]; 
  }
  _wbc_data->contact_state = cMPCOld->contact_state;
  //wbc计算
  _wbc_ctrl->run(_wbc_data, *this->_data);
}
/**
 * 判断动作是否完成,Busy状态下不允许切换状态
 * locomotion状态下,什么时候适合切换到站立状态呢?
 * 支撑腿不需要考虑,任何时候都适合
 * 摆动腿的摆动动作刚刚开始,或者即将结束的时候,适合切换
 * 因此应该让x=sum(abs(swing_state-0.5)),尽量大
 * 对x求导得到dx,当首次dx<0,既x为减函数时允许切换
*/
template <typename T>
bool FSM_State_Locomotion<T>::isBusy() {

  float x,dx;
  static float pre_x  = 0.0;
  static float pre_dx = 0.0;
  float filter = 0.1;

  Vec4<float> s = cMPCOld->swing_state;

  x = fabs(s[0] - 0.5) + fabs(s[1] - 0.5) + fabs(s[2] - 0.5) + fabs(s[3] - 0.5);
  
  dx = x - pre_x;
  dx = dx*filter + pre_dx*(1-filter);

  bool busyFlag = true;
  if((dx<0)&&(pre_dx>=0)) busyFlag = false;
  pre_x  = x;
  pre_dx = dx;
  
  return busyFlag;
}
// template class FSM_State_Locomotion<double>;
template class FSM_State_Locomotion<float>;
