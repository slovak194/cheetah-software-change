#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include <ControlParameters/RobotParameters.h>
#include <MIT_UserParameters.h>
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"
#include "SimUtilities/GamepadCommand.h"
/**
 *
 */
template <typename T>
struct ControlFSMData {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GamepadCommand* _gamepadCommand;
  Quadruped<T>* _quadruped;
  StateEstimatorContainer<T>* _stateEstimator;
  LegController<T>* _legController;
  RobotControlParameters* controlParameters;
  MIT_UserParameters* userParameters;
  VisualizationData* visualizationData;
};

template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

#endif  // CONTROLFSM_H