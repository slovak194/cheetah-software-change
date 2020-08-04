#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include <Controllers/convexMPC/ConvexMPCLocomotion.h>
#include "FSM_State.h"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;
/**
 *
 */
template <typename T>
class FSM_State_Locomotion : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

 private:
  ConvexMPCLocomotion* cMPCOld;
  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  // Parses contact specific controls to the leg controller
  void LocomotionControlStep();

  bool locomotionSafe();
};

#endif  // FSM_STATE_LOCOMOTION_H
