/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"
#include "FSM_State.h"
#include "gamepad/Gamepad.hpp"
#include "string.h"

/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param _quadruped the quadruped information
 * @param _stateEstimator contains the estimated states
 * @param _legController interface to the leg controllers
 * @param _gaitScheduler controls scheduled foot contact modes
 * @param _desiredStateCommand gets the desired COM state trajectories
 * @param controlParameters passes in the control parameters from the GUI
 */
template <typename T>
ControlFSM<T>::ControlFSM(Gamepad* gamepad,
                          Quadruped<T>* _quadruped,
                          StateEstimatorContainer<T>* _stateEstimator,
                          LegController<T>* _legController,
                          RobotControlParameters* controlParameters,
                          VisualizationData* visualizationData,
                          MIT_UserParameters* userParameters)
{
  // Add the pointers to the ControlFSMData struct
  data._gamepad = gamepad;
  data._quadruped = _quadruped;
  data._stateEstimator = _stateEstimator;
  data._legController = _legController;
  data.controlParameters = controlParameters;
  data.visualizationData = visualizationData;
  data.userParameters = userParameters;

  // Initialize and add all of the FSM States to the state list
  statesList.invalid = nullptr;
  statesList.passive = new FSM_State_Passive<T>(&data);
  statesList.sitDown = new FSM_State_SitDown<T>(&data);
  statesList.standUp = new FSM_State_StandUp<T>(&data);
  statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);
  statesList.locomotion = new FSM_State_Locomotion<T>(&data);

  safetyChecker = new SafetyChecker<T>(&data);

  // Initialize the FSM with the Passive FSM State
  initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state.
 */
template <typename T>
void ControlFSM<T>::initialize() {
  // 上电后,passive
  currentState = statesList.passive;
  currentState->onEnter();
  nextState = currentState;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
void ControlFSM<T>::runFSM() {
  if(nextState == currentState){

    switch(currentState->stateName){
      // PASSIVE  =>  STAND_UP
      case FSM_StateName::PASSIVE:
        if(data._gamepad->get().leftBumper)
            nextState = statesList.standUp;
        break;
      // STAND_UP  =>  LOCOMOTION
      // STAND_UP  =>  SIT_DOWN
      // STAND_UP  =>  BALANCE_STAND
      case FSM_StateName::STAND_UP:
        if(currentState->isBusy()) break;
        if(data._gamepad->get().x)
            nextState = statesList.locomotion;
        else if(data._gamepad->get().rightBumper)
            nextState = statesList.sitDown;
        else if(data._gamepad->get().y)
            nextState = statesList.balanceStand;
        break;
      // LOCOMOTION  =>  STAND_UP
      case FSM_StateName::LOCOMOTION:
        if(data._gamepad->get().b)
            nextState = statesList.standUp;
        break;
      //  BALANCE_STAND  =>  STAND_UP
      case FSM_StateName::BALANCE_STAND:
        if(currentState->isBusy()) break;
        if(data._gamepad->get().b)
            nextState = statesList.standUp;
        break;
      //  SIT_DOWN  =>  PASSIVE
      case FSM_StateName::SIT_DOWN:
        if(currentState->isBusy()) break;
        if(data._gamepad->get().rightBumper)
            nextState = statesList.passive;
        break;
      default: break;
    }
  }

  //状态切换
  if(nextState != currentState)
  {
    if(!currentState->isBusy())
    {
      currentState = nextState;
      currentState->onEnter();
    }
  }

  safetyPreCheck();
  currentState->run();
  safetyPostCheck();
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 * 安全预检
 * @return the appropriate operating mode
 */
template <typename T>
bool ControlFSM<T>::safetyPreCheck() {
  // Check for safe orientation if the current state requires it
  if (currentState->checkSafeOrientation) {
    if (!safetyChecker->checkSafeOrientation()) {
      std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;
    }
  }
  return true;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */
template <typename T>
bool ControlFSM<T>::safetyPostCheck() {
  // Check for safe desired foot positions
  // 检测足底期望位置是否安全
  if (currentState->checkPDesFoot) {
    safetyChecker->checkPDesFoot();
  }
  // Check for safe desired feedforward forces
  // 检测期望足底前馈力是否安全
  if (currentState->checkForceFeedForward) {
    safetyChecker->checkForceFeedForward();
  }
  return true;
}

template class ControlFSM<float>;
