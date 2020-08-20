/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <cassert>
#include <iostream>

#include "HardwareBridge.h"
#include "SimulationBridge.h"
#include "WebotsBridge.hpp"
#include "main_helper.h"
#include "RobotController.h"

/*!
 * Print a message describing the command line flags for the robot program
 */
void printUsage() {
  printf(
      "Usage: controller_name [sim-or-robot-or-webots]\n"
      "\t  sim-or-robot: s for sim, r for robot, w for webots simulator\n");
}

/*!
 * Setup and run the given robot controller
 * 启动并运行给定的机器人控制器
 */
int main_helper(int argc, char** argv, RobotController* ctrl) {
  // 若参数数量不对,打印使用说明
  if (argc != 2) {
    printUsage();
    return EXIT_FAILURE;
  }

  // 判断是使用仿真模式还是实物机器人模式
  if (argv[1][0] == 's')
  {
    SimulationBridge simulationBridge( ctrl);
    simulationBridge.run();
  } 
  else if (argv[1][0] == 'r') 
  {
    MiniCheetahHardwareBridge hw(ctrl, true);
    hw.run();
  } 
  else if (argv[1][0] == 'w') 
  {
    MiniCheetahWebotsBridge miniCheetahWebotsBridge(ctrl);
    miniCheetahWebotsBridge.run();
  }
   else 
  {
    printUsage();
    return EXIT_FAILURE;
  }
  return 0;
}
