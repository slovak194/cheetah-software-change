/*!
 * @file HardwareBridge.h
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */

#ifndef PROJECT_HARDWAREBRIDGE_H
#define PROJECT_HARDWAREBRIDGE_H

#ifdef linux 

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

#include <string>
#include <lcm-cpp.hpp>
#include <lord_imu/LordImu.h>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "microstrain_lcmt.hpp"
#include "ecat_command_t.hpp"
#include "ecat_data_t.hpp"



/*!
 * Interface between robot and hardware
 * 机器人与硬件之间的接口
 */
class HardwareBridge {
 public:
  HardwareBridge(RobotController* robot_ctrl)
      : statusTask(&taskManager, 0.5f),
        _interfaceLCM(getLcmUrl(255)),      //初始化lcm实例,并链接到特殊LCM网络.可用good()函数查询是否成功
        _visualizationLCM(getLcmUrl(255)) {
    _controller = robot_ctrl;
    _userControlParameters = robot_ctrl->getUserControlParameters();
        }
  void prefaultStack();
  void setupScheduler();
  void initError(const char* reason, bool printErrno = false);
  void initCommon();
  ~HardwareBridge() { delete _robotRunner; }
  void handleGamepadLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                        const gamepad_lcmt* msg);

  void handleInterfaceLCM();
  void handleControlParameter(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const control_parameter_request_lcmt* msg);

  void publishVisualizationLCM();

 protected:
  PeriodicTaskManager taskManager;                   //任务管理器
  PrintTaskStatus statusTask;                        //打印任务状态类,似乎取消了
  GamepadCommand _gamepadCommand;                    //游戏手柄数据
  VisualizationData _visualizationData;              //可视化数据,调试用
  CheetahVisualization _mainCheetahVisualization;    //在仿真环境上绘制当前机器人?
  lcm::LCM _interfaceLCM;                            //lcm接口
  lcm::LCM _visualizationLCM;                        //用于可视化的lcm接口
  control_parameter_respones_lcmt _parameter_response_lcmt;  //控制参数响应lcm数据类型
  SpiData _spiData;                                  //spi过来的数据
  SpiCommand _spiCommand;                            //spi命令

  TiBoardCommand _tiBoardCommand[4];                 //Tiboard命令
  TiBoardData _tiBoardData[4];

  bool _firstRun = true;                             //首次运行标志
  RobotRunner* _robotRunner = nullptr;               //机器人运行器,比较重要
  RobotControlParameters _robotParams;               //机器人控制参数
  u64 _iterations = 0;                               //迭代器,
  std::thread _interfaceLcmThread;                   //lcm接口线程
  volatile bool _interfaceLcmQuit = false;     
  RobotController* _controller = nullptr;
  ControlParameters* _userControlParameters = nullptr;
};

/*!
 * Interface between robot and hardware specialized for Mini Cheetah
 * 专门用于mini cheetah的机器人与硬件之间的接口
 */
class MiniCheetahHardwareBridge : public HardwareBridge {
 public:
  MiniCheetahHardwareBridge(RobotController* rc, bool load_parameters_from_file);
  void runSpi();
  void initHardware();
  void run();
  void runMicrostrain();
  void logMicrostrain();
  void abort(const std::string& reason);
  void abort(const char* reason);

 private:
  VectorNavData _vectorNavData;  //陀螺仪数据
  lcm::LCM _spiLcm;
  lcm::LCM _microstrainLcm;
  std::thread _microstrainThread;
  LordImu _microstrainImu;       //陀螺仪
  microstrain_lcmt _microstrainData;
  bool _microstrainInit = false;
  bool _load_parameters_from_file;
};
#endif // END of #ifdef linux
#endif  // PROJECT_HARDWAREBRIDGE_H
