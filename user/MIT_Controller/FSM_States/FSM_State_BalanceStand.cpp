/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "FSM_State_BalanceStand.h"
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND,"BALANCE_STAND") {
  // Set the pre controls safety checks
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;
  // Initialize
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();
  //设置Q2权重
  _wbc_ctrl->setFloatingBaseWeight(1000.);
}

template <typename T>
void FSM_State_BalanceStand<T>::onEnter() {
  
  printf("[FSM_State_BalanceStand] onEnter...\n");
  _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;

  _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
  _body_weight = this->_data->_quadruped->_bodyMass * 9.81;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_BalanceStand<T>::run() {
  Vec4<T> contactState;
  contactState<< 0.5, 0.5, 0.5, 0.5;
  this->_data->_stateEstimator->setContactPhase(contactState);
  BalanceStandStep();
}

/**
 * Calculate the commands for the leg controllers for each of the feet.
 */
template <typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep() {

  _wbc_data->pBody_des = _ini_body_pos;
  _wbc_data->vBody_des.setZero();
  _wbc_data->aBody_des.setZero();
  // Orientation
  static Vec3<T> pre_des_Ori(0,0,0);
  Vec3<T> des_Ori(
  0.6*this->_data->_gamepadCommand->leftStickAnalog[0],
  0.2*this->_data->_gamepadCommand->leftStickAnalog[1],
  0.4*(this->_data->_gamepadCommand->leftTriggerAnalog - this->_data->_gamepadCommand->rightTriggerAnalog)
  );
  T filter(0.007);
  des_Ori = des_Ori*filter + pre_des_Ori*(1-filter);
  pre_des_Ori = des_Ori;
  _wbc_data->pBody_RPY_des = des_Ori;
  _wbc_data->vBody_Ori_des.setZero();
  // height
  _wbc_data->pBody_des[2] -= 0.5*sin(fabs(des_Ori[0]))*
                             (this->_data->_quadruped->_bodyWidth + this->_data->_quadruped->_abadLinkLength);
  _wbc_data->pBody_des[2] -= 0.5*sin(fabs(des_Ori[1]))* this->_data->_quadruped->_bodyLength;  
    

  for(size_t i(0); i<4; ++i){
    _wbc_data->pFoot_des[i].setZero();
    _wbc_data->vFoot_des[i].setZero();
    _wbc_data->aFoot_des[i].setZero();
    _wbc_data->Fr_des[i].setZero();
    _wbc_data->Fr_des[i][2] = _body_weight/4.;
    _wbc_data->contact_state[i] = true;
  }

  _wbc_ctrl->run(_wbc_data, *this->_data);
}
/**
 * 判断动作是否完成,Busy状态下不允许切换状态
*/
template <typename T>
bool FSM_State_BalanceStand<T>::isBusy() {
  if(
    fabs((this->_data->_gamepadCommand->leftStickAnalog[0] > 5))||
    fabs((this->_data->_gamepadCommand->leftStickAnalog[1] > 5))||
    fabs((this->_data->_gamepadCommand->rightStickAnalog[0] > 5))||
    fabs((this->_data->_gamepadCommand->rightStickAnalog[1] > 5))||
    fabs((this->_data->_gamepadCommand->rightTriggerAnalog > 5))||
    fabs((this->_data->_gamepadCommand->leftTriggerAnalog > 5))
  )
  return true; //不能在操作过程中切换状态

  return false;
}
// template class FSM_State_BalanceStand<double>;
template class FSM_State_BalanceStand<float>;
