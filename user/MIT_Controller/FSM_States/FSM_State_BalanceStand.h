#ifndef FSM_STATE_BALANCESTAND_H
#define FSM_STATE_BALANCESTAND_H

#include "FSM_State.h"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;

/**
 *
 */
template <typename T>
class FSM_State_BalanceStand : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData);
  void onEnter() override;
  void run();
  bool isBusy();

 private:

  // Parses contact specific controls to the leg controller
  void BalanceStandStep();

  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  Vec3<T> _ini_body_pos;
  Vec3<T> _ini_body_ori_rpy;
  T _body_weight;
};

#endif  // FSM_STATE_BALANCESTAND_H
