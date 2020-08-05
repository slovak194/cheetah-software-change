/*============================= Stand Up ==============================*/
/**
 */
#include "FSM_State_StandUp.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
_ini_foot_pos(4){
  //站立状态是确定的,不需要安全检查
  this->turnOffAllSafetyChecks();
}

template <typename T>
void FSM_State_StandUp<T>::onEnter() {

  printf("[FSM_State_StandUp] onEnter...\n");
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
void FSM_State_StandUp<T>::run() {

  iter++;
  T stand_up_height = this->_data->userParameters->stand_up_height;
  T t               = (iter * this->_data->controlParameters->controller_dt)/
                      (this->_data->userParameters->stand_up_time);

  if (t > 1.){ t = 1.; }

  for(int i = 0; i < 4; i++) {
    this->_data->_legController->commands[i].kpCartesian = Vec3<T>(1000, 1000, 1000).asDiagonal();
    this->_data->_legController->commands[i].kdCartesian = Vec3<T>(16, 16, 40).asDiagonal();

    //x,y保持原来的位置
    this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
    //z变成stand_up_height
    this->_data->_legController->commands[i].pDes[2] = T_Curve(_ini_foot_pos[i][2], -stand_up_height, t);
    }
}
/**
 * 判断动作是否完成,Busy状态下不允许切换状态
*/
template <typename T>
bool FSM_State_StandUp<T>::isBusy() {

  T t = (iter * this->_data->controlParameters->controller_dt)/
        (this->_data->userParameters->stand_up_time);

  if(t < 1.0f)
    return true;
  else
    return false;
}
// template class FSM_State_StandUp<double>;
template class FSM_State_StandUp<float>;
