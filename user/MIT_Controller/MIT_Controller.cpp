#include "MIT_Controller.hpp"
#include "string.h"
MIT_Controller::MIT_Controller():RobotController("mit_ctrl"){ }

void MIT_Controller::initializeController() {

  // 初始化状态机
  _controlFSM = new ControlFSM<float>(_gamepad,_quadruped, _stateEstimator,
                                      _legController,
                                       _controlParameters, _visualizationData, 
                                       &userParameters);
}

void MIT_Controller::runController() {

  // 运行状态机
  _controlFSM->runFSM();
}


