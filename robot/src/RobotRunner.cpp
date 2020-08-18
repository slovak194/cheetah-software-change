/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah
 */

#include <unistd.h>

#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/cheetah.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
//#include "rt/rt_interface_lcm.h"

RobotRunner::RobotRunner(RobotController* robot_ctrl, 
    PeriodicTaskManager* manager, 
    float period, std::string name):
  PeriodicTask(manager, period, name),
  _lcm(getLcmUrl(255)) {
    _robot_ctrl = robot_ctrl;
  }

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init() {
  printf("[RobotRunner] initialize\n");

  // Build the appropriate Quadruped object
  // 建立机器人各项物理参数
  _quadruped = buildCheetah<float>();

  // Initialize the model and robot data
  _model = _quadruped.buildModel();

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(
      cheaterState, vectorNavData, _legController->datas,
      &_stateEstimate, controlParameters);
  initializeStateEstimator(false);

  // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_stateEstimator = _stateEstimator;
  _robot_ctrl->_stateEstimate = &_stateEstimate;
  _robot_ctrl->_visualizationData= visualizationData;
  _robot_ctrl->_gamepadCommand = driverCommand;
  _robot_ctrl->_controlParameters = controlParameters;

  _robot_ctrl->initializeController();

}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner::run() {
  // Run the state estimator step
  //_stateEstimator->run(cheetahMainVisualization);
  _stateEstimator->run();
  
  //cheetahMainVisualization->p = _stateEstimate.position;
  visualizationData->clear();

  // Update the data from the robot
  setupStep();

  static int count_ini(0);
  ++count_ini;
  if (count_ini < 50) {
    _legController->setEnabled(false);
  } else {
    _legController->setEnabled(true);
    // Controller
    _robot_ctrl->runController();
    cheetahMainVisualization->p = _stateEstimate.position;
    // Update Visualization
    _robot_ctrl->updateVisualization();
    cheetahMainVisualization->p = _stateEstimate.position;
  }
  // Visualization (will make this into a separate function later)
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      cheetahMainVisualization->q[leg * 3 + joint] =
        _legController->datas[leg].q[joint];
    }
  }
  cheetahMainVisualization->p.setZero();
  cheetahMainVisualization->p = _stateEstimate.position;
  cheetahMainVisualization->quat = _stateEstimate.orientation;

  // Sets the leg controller commands for the robot appropriate commands
  finalizeStep();
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner::setupStep() {
  // Update the leg data
  _legController->updateData(spiData);

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);

  // state estimator
  // check transition to cheater mode:
  if (!_cheaterModeEnabled && controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning to Cheater Mode...\n");
    initializeStateEstimator(true);
    // todo any configuration
    _cheaterModeEnabled = true;
  }

  // check transition from cheater mode:
  if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning from Cheater Mode...\n");
    initializeStateEstimator(false);
    // todo any configuration
    _cheaterModeEnabled = false;
  }

  // todo safety checks, sanity checks, etc...
}

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 */
void RobotRunner::finalizeStep() {
  _legController->updateCommand(spiCommand);
  
  _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
  _stateEstimate.setLcm(state_estimator_lcm);
  _lcm.publish("leg_control_command", &leg_control_command_lcm);
  _lcm.publish("leg_control_data", &leg_control_data_lcm);
  _lcm.publish("state_estimator", &state_estimator_lcm);
  _iterations++;
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);
  if (cheaterMode) {
    _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
    _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
  } else {
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
  }
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
}

void RobotRunner::cleanup() {}
