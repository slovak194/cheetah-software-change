/*============================= FSM State =============================*/
/**
 * FSM State base class
 */

#include "FSM_State.h"

/**
 * Constructor for the FSM State class.
 *
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name
 * @param stateStringIn the string name of the current FSM state
 */
template <typename T>
FSM_State<T>::FSM_State(ControlFSMData<T>* _controlFSMData,
                        FSM_StateName stateNameIn, std::string stateStringIn)
    : _data(_controlFSMData),
      stateName(stateNameIn),
      stateString(stateStringIn) {
  std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn
            << std::endl;
}
/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
template <typename T>
void FSM_State<T>::turnOnAllSafetyChecks() {
  // Pre controls safety checks
  checkSafeOrientation = true;  // check roll and pitch

  // Post control safety checks
  checkPDesFoot = true;          // do not command footsetps too far
  checkForceFeedForward = true;  // do not command huge forces
  checkLegSingularity = true;    // do not let leg
}

/**
 *
 */
template <typename T>
void FSM_State<T>::turnOffAllSafetyChecks() {
  // Pre controls safety checks
  checkSafeOrientation = false;  // check roll and pitch

  // Post control safety checks
  checkPDesFoot = false;          // do not command footsetps too far
  checkForceFeedForward = false;  // do not command huge forces
  checkLegSingularity = false;    // do not let leg
}

// template class FSM_State<double>;
template class FSM_State<float>;
