/*! @file main.cpp
 *  @brief Main function for simulator
 */

#include "Collision/CollisionPlane.h"
#include "DrawList.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/Quadruped.h"
#include "Graphics3D.h"
#include "SimControlPanel.h"
#include "Simulation.h"
#include "Utilities/utilities.h"
#include "Utilities/SegfaultHandler.h"

#include <QApplication>
#include <QSurfaceFormat>

#include <stdio.h>
#include <unistd.h>
#include <thread>

/*!
 * Setup QT and run a simulation
 * 启动QT并且运行仿真器
 */
int main(int argc, char *argv[]) {
  install_segfault_handler(nullptr);
  // set up Qt
  QApplication a(argc, argv);

  // open simulator UI
  SimControlPanel panel;
  panel.show();

  // run the Qt program
  a.exec();

  return 0;
}
