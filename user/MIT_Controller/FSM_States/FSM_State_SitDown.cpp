/*============================= Sit Down ==============================*/
/** 执行之后，机器人趴下
 */
#include "Math/Interpolation.h"
#include "FSM_State_SitDown.h"
#define PI (3.141592654f)
/**
 * 构造函数
 *
 * @param _controlFSMData 这里面保存了相关参数
 */
template <typename T>
FSM_State_SitDown<T>::FSM_State_SitDown(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::SIT_DOWN, "SIT_DOWN"),
_ini_joint_pos(4),_end_joint_pos(4){
  //站立状态是确定的,不需要安全检查
  this->turnOffAllSafetyChecks();
}


template <typename T>
void FSM_State_SitDown<T>::onEnter() {

  printf("[FSM_State_SitDown] onEnter...\n");
  // Reset iteration counter
  iter = 0;
  //make theta
  float l1 = this->_data->_quadruped->_hipLinkLength;
  float l2 = this->_data->_quadruped->_kneeLinkLength;
  float h  = this->_data->userParameters->sit_down_height;

  float theta1 = -acosf((l1*l1+h*h-l2*l2)/(2*l1*h));
  float theta2 = PI - acosf((l1*l1+l2*l2-h*h)/(2*l1*l2));

  for(size_t leg(0); leg<4; ++leg){
    _ini_joint_pos[leg] = this->_data->_legController->datas[leg].q;

    _end_joint_pos[leg][0] = 0;
    _end_joint_pos[leg][1] = theta1;
    _end_joint_pos[leg][2] = theta2;
  }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_SitDown<T>::run() {

  iter++;

  T sit_down_time = this->_data->userParameters->sit_down_time;
  T t               = (iter * this->_data->controlParameters->controller_dt)/sit_down_time;

  if (t > 1.){ t = 1.; }

  Vec3<T> kp(this->_data->userParameters->Kp_stand[0],
             this->_data->userParameters->Kp_stand[1],
             this->_data->userParameters->Kp_stand[2]);
  Vec3<T> kd(this->_data->userParameters->Kd_stand[0],
             this->_data->userParameters->Kd_stand[1],
             this->_data->userParameters->Kd_stand[2]);
  for(int leg = 0; leg < 4; leg++) {
    this->_data->_legController->commands[leg].kpJoint = kp.asDiagonal();
    this->_data->_legController->commands[leg].kdJoint = kd.asDiagonal();
  //数据插值
  this->_data->_legController->commands[leg].qDes
   = Interpolate::cubicBezier<Vec3<T>>(_ini_joint_pos[leg], _end_joint_pos[leg], t);
  this->_data->_legController->commands[leg].qdDes
  = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_ini_joint_pos[leg], _end_joint_pos[leg], t) / sit_down_time;
      //debug printf
      /*
      if(leg == 3)
      {
        for(int j=0;j<3;j++)
        printf("%d：%.3f  ",j, this->_data->_legController->commands[leg].qDes[j]-this->_data->_legController->datas[leg].q[j]);
        printf("\n");
      }
      */
    }
}

/**
 * 判断动作是否完成,Busy状态下不允许切换状态
*/
template <typename T>
bool FSM_State_SitDown<T>::isBusy() {

  T t = (iter * this->_data->controlParameters->controller_dt)/
        (this->_data->userParameters->sit_down_time);

  if(t < 1.0f)
    return true;
  else
    return false;
}
// template class FSM_State_StandUp<double>;
template class FSM_State_SitDown<float>;
