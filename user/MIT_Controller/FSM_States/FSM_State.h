#ifndef FSM_State_H
#define FSM_State_H

#include <stdio.h>

#include "ControlFSMData.h"

// Normal robot states
#define K_INVALID         (0)
#define K_PASSIVE         (1)
#define K_STAND_UP        (2)
#define K_SIT_DOWN        (3)
#define K_LOCOMOTION      (4)
#define K_BALANCE_STAND   (5)

/**
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum class FSM_StateName {
  INVALID,
  PASSIVE,
  STAND_UP,
  SIT_DOWN,
  LOCOMOTION,
  BALANCE_STAND
};

template <typename T>
class FSM_State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Generic constructor for all states
  FSM_State(ControlFSMData<T>* _controlFSMData, FSM_StateName stateNameIn,
            std::string stateStringNameI);

  // Behavior to be carried out when entering a state
  virtual void onEnter() = 0;// {}

  // Run the normal behavior for the state
  virtual void run() = 0; //{}
  
  //在busy状态下,不允许进行状态切换
  virtual bool isBusy() = 0;
  //
  void turnOnAllSafetyChecks();
  void turnOffAllSafetyChecks();

  // Holds all of the relevant control data
  ControlFSMData<T>* _data;

  // FSM State info
  FSM_StateName stateName;          // enumerated name of the current state

  // Pre controls safety checks
  bool checkSafeOrientation = false;  // check roll and pitch

  // Post control safety checks
  bool checkPDesFoot = false;          // do not command footsetps too far
  bool checkForceFeedForward = false;  // do not command huge forces
  bool checkLegSingularity = false;    // do not let leg 奇异位置
};

#endif  // FSM_State_H
