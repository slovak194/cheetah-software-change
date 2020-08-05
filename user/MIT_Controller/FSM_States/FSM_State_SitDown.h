#ifndef FSM_STATE_SITDOWN_H
#define FSM_STATE_SITDOWN_H

#include "FSM_State.h"

/**
 *
 */
template <typename T>
class FSM_State_SitDown : public FSM_State<T> {
 public:
  FSM_State_SitDown(ControlFSMData<T>* _controlFSMData);
  void onEnter();
  void run();
  bool isBusy();

 private:
  // Keep track of the control iterations
  int iter = 0;
  std::vector< Vec3<T> > _ini_foot_pos;
};

#endif  // FSM_STATE_SITDOWN_H
