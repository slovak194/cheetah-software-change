#ifndef FSM_STATE_STANDUP_H
#define FSM_STATE_STANDUP_H

#include "FSM_State.h"

/**
 *
 */
template <typename T>
class FSM_State_StandUp : public FSM_State<T> {
 public:
  FSM_State_StandUp(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  bool isBusy();

 private:
  // 迭代器，进入状态后迭代了多少个dt周期
  int iter = 0;
  std::vector< Vec3<T> > _ini_joint_pos;
  std::vector< Vec3<T> > _end_joint_pos;
};

#endif  // FSM_STATE_STANDUP_H
