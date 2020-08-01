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
  transitionData.zero();
  std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn
            << std::endl;
}

/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param qDes desired joint position
 * @param dqDes desired joint velocity
 */
template <typename T>
void FSM_State<T>::jointPDControl(
    int leg, Vec3<T> qDes, Vec3<T> qdDes) {
  //硬编码,后面需要改动
  kpMat << 80, 0, 0, 0, 80, 0, 0, 0, 80;
  kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  _data->_legController->commands[leg].kpJoint = kpMat;
  _data->_legController->commands[leg].kdJoint = kdMat;

  _data->_legController->commands[leg].qDes = qDes;
  _data->_legController->commands[leg].qdDes = qdDes;
}

/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param pDes desired foot position
 * @param vDes desired foot velocity
 * @param kp_cartesian P gains
 * @param kd_cartesian D gains
 */
template <typename T>
void FSM_State<T>::cartesianImpedanceControl(int leg, Vec3<T> pDes,
                                             Vec3<T> vDes,
                                             Vec3<double> kp_cartesian,
                                             Vec3<double> kd_cartesian) {
  _data->_legController->commands[leg].pDes = pDes;
  // Create the cartesian P gain matrix
  kpMat << kp_cartesian[0], 0, 0, 0,kp_cartesian[1], 0, 0, 0,kp_cartesian[2];
  _data->_legController->commands[leg].kpCartesian = kpMat;

  _data->_legController->commands[leg].vDes = vDes;
  // Create the cartesian D gain matrix
  kdMat << kd_cartesian[0], 0, 0, 0, kd_cartesian[1], 0, 0, 0, kd_cartesian[2];
  _data->_legController->commands[leg].kdCartesian = kdMat;
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
