/*============================== Passive ==============================*/
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

#include "FSM_State_Passive.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE") {
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_Passive<T>::onEnter() {

  printf("[FSM_State_Passive] onEnter...\n");
}

/**
 * Busy状态下不允许切换状态
*/
template <typename T>
bool FSM_State_Passive<T>::isBusy() {

  return false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Passive<T>::run() {

  for(int i = 0; i < 4; i++) {
  this->_data->_legController->commands[i].zero();
  }
}
// template class FSM_State_Passive<double>;
template class FSM_State_Passive<float>;
