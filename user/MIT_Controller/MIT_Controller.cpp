#include "MIT_Controller.hpp"
#include "string.h"
MIT_Controller::MIT_Controller():RobotController("mit_ctrl"){ }

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController() {

  // Initialize a new ContactEstimator object
  //_contactEstimator = new ContactEstimator<double>();
  //_contactEstimator->initialize();

  // Initializes the Control FSM with all the required data
  // 使用需要的数据,初始化状态机
  _controlFSM = new ControlFSM<float>(_gamepadCommand,_quadruped, _stateEstimator,
                                      _legController,
                                       _controlParameters, _visualizationData, 
                                       &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 * 使用ControlFSM 计算用于腿部控制器的命令
 */
void MIT_Controller::runController() {

  // Run the Control FSM code
  // 运行控制
  _controlFSM->runFSM();
}


