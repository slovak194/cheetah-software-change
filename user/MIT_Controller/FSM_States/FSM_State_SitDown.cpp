/*============================= Sit Down ==============================*/
/**
 * Transitionary state that is called for the robot to Sit Down into
 * balance control mode.
 */

#include "FSM_State_SitDown.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_SitDown<T>::FSM_State_SitDown(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::SIT_DOWN, "SIT_DOWN"),
_ini_foot_pos(4){
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_SitDown<T>::onEnter() {

  printf("[FSM_State_SitDown] onEnter...\n");
  // Reset iteration counter
  iter = 0;

  for(size_t leg(0); leg<4; ++leg){
    _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
  }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_SitDown<T>::run() {

  T hMax = this->_data->userParameters->sit_down_height;
  T progress = (iter * this->_data->controlParameters->controller_dt)/
                (this->_data->userParameters->sit_down_time);

  if (progress > 1.){ progress = 1.; }

  for(int i = 0; i < 4; i++) {
    this->_data->_legController->commands[i].kpCartesian = Vec3<T>(1000, 1000, 1000).asDiagonal();
    this->_data->_legController->commands[i].kdCartesian = Vec3<T>(16, 16, 40).asDiagonal();

    this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
    this->_data->_legController->commands[i].pDes[2] = 
      progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];
    }
}

// template class FSM_State_StandUp<double>;
template class FSM_State_SitDown<float>;
