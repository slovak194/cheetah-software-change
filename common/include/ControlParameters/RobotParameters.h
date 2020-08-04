/*! @file RobotParameters.cpp
 *  @brief Declaration of various robot parameters
 *
 *  This class contains all the ControlParameters which are shared between all robot controllers
 *  Currently there are some controlParameters that are specific to the MIT controllers here,
 *  but these will be moved in the future
 */

#ifndef PROJECT_ROBOTPARAMETERS_H
#define PROJECT_ROBOTPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

/*!
 * ControlParameters shared between all robot controllers
 */
class RobotControlParameters : public ControlParameters {
 public:

  /*!
   * Construct RobotControlParameters
   */
  RobotControlParameters()
      : ControlParameters("robot-parameters"),
        INIT_PARAMETER(controller_dt),
        INIT_PARAMETER(cheater_mode),
        INIT_PARAMETER(imu_process_noise_position),
        INIT_PARAMETER(imu_process_noise_velocity),
        INIT_PARAMETER(foot_process_noise_position),
        INIT_PARAMETER(foot_sensor_noise_position),
        INIT_PARAMETER(foot_sensor_noise_velocity),
        INIT_PARAMETER(foot_height_sensor_noise){}

  DECLARE_PARAMETER(double, controller_dt)

  // state estimator
  DECLARE_PARAMETER(s64, cheater_mode)
  DECLARE_PARAMETER(double, imu_process_noise_position)
  DECLARE_PARAMETER(double, imu_process_noise_velocity)
  DECLARE_PARAMETER(double, foot_process_noise_position)
  DECLARE_PARAMETER(double, foot_sensor_noise_position)
  DECLARE_PARAMETER(double, foot_sensor_noise_velocity)
  DECLARE_PARAMETER(double, foot_height_sensor_noise)

};

#endif  // PROJECT_ROBOTPARAMETERS_H
