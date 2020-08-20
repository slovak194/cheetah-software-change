/*!
 * @file RobotController.h
 * @brief Parent class of user robot controllers.
 * This is an interface between the control code and the common hardware code
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "gamepad/Gamepad.hpp"
#include "Controllers/LegController.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Controllers/StateEstimatorContainer.h"
#include "SimUtilities/VisualizationData.h"


/*!
 * Parent class of user robot controllers
 */
class RobotController{
  friend class RobotRunner;
public:
  RobotController(const std::string& name) : ctrl_name(name){}
  virtual ~RobotController(){}

  virtual void initializeController() = 0;
/**
 * Called one time every control loop 
 */
  virtual void runController() = 0;
  virtual void updateVisualization() = 0;
  virtual ControlParameters* getUserControlParameters() = 0;
  virtual void Estop() {}
  std::string ctrl_name;

protected:
  Quadruped<float>* _quadruped = nullptr;
  FloatingBaseModel<float>* _model = nullptr;
  LegController<float>* _legController = nullptr;
  StateEstimatorContainer<float>* _stateEstimator = nullptr;
  StateEstimate<float>* _stateEstimate = nullptr;
  Gamepad* _gamepad = nullptr;
  RobotControlParameters* _controlParameters = nullptr;

  VisualizationData* _visualizationData = nullptr;
};

#endif
