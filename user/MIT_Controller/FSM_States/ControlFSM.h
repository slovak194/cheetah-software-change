#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>

// Contains all of the control related data
#include "ControlFSMData.h"

// Checks the robot state and commands for safety
#include "SafetyChecker.h"

// FSM States
#include "../FSM_States/FSM_State.h"
#include "../FSM_States/FSM_State_BalanceStand.h"
#include "../FSM_States/FSM_State_Locomotion.h"
#include "../FSM_States/FSM_State_Passive.h"
#include "../FSM_States/FSM_State_SitDown.h"
#include "../FSM_States/FSM_State_StandUp.h"
#include "gamepad/Gamepad.hpp"

/**
 *
 */
template <typename T>
struct FSM_StatesList {
  FSM_State<T>* invalid;
  FSM_State_Passive<T>* passive;
  FSM_State_StandUp<T>* standUp;
  FSM_State_SitDown<T>* sitDown;
  FSM_State_BalanceStand<T>* balanceStand;
  FSM_State_Locomotion<T>* locomotion;
};

/**
 * Control FSM handles the FSM states from a higher level
 */
template <typename T>
class ControlFSM {
 public:
  ControlFSM(
             Gamepad* gamepad,
             Quadruped<T>* _quadruped,
             StateEstimatorContainer<T>* _stateEstimator,
             LegController<T>* _legController,
             RobotControlParameters* controlParameters,
             VisualizationData* visualizationData,
             MIT_UserParameters* userParameters);

  // Initializes the Control FSM instance
  void initialize();

  // Runs the FSM logic and handles the state transitions and normal runs
  void runFSM();

  // This will be removed and put into the SafetyChecker class
  bool safetyPreCheck();
  bool safetyPostCheck();

  // Contains all of the control related data
  ControlFSMData<T> data;

  // FSM state information
  FSM_StatesList<T> statesList;  // holds all of the FSM States
  FSM_State<T>* currentState;    // current FSM state
  FSM_State<T>* nextState;       // next FSM state

  // Checks all of the inputs and commands for safety
  SafetyChecker<T>* safetyChecker;
};

#endif  // CONTROLFSM_H
