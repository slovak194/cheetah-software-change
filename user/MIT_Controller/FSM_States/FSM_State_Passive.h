#ifndef FSM_STATE_PASSIVE_H
#define FSM_STATE_PASSIVE_H

#include "FSM_State.h"

template <typename T>
class FSM_State_Passive : public FSM_State<T> {
 public:
  FSM_State_Passive(ControlFSMData<T>* _controlFSMData);
  void onEnter();
  void run();
  bool isBusy();
};

#endif  // FSM_STATE_PASSIVE_H
