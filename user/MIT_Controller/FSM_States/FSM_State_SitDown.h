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

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

 private:
  // Keep track of the control iterations
  int iter = 0;
  std::vector< Vec3<T> > _ini_foot_pos;
};

#endif  // FSM_STATE_SITDOWN_H
