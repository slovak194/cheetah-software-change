/*!
 * @file WebotsBridge.h
 * @brief Interface between robot code and webots simulator
 */

#ifndef PROJECT_WEBOTSBRIDGE_H
#define PROJECT_WEBOTSBRIDGE_H

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

#include <string>
#include <lcm-cpp.hpp>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "controller2webots_t.hpp"
#include "webots2controller_t.hpp"
#include "gamepad/Gamepad.hpp"
/*!
 * Interface between robot code and webots simulator
 * 机器人与webots仿真器之间的接口
 */
class WebotsBridge {
 public:
  WebotsBridge(RobotController* robot_ctrl)
      : _interfaceLCM(getLcmUrl(255)),      //初始化lcm实例,并链接到特殊LCM网络.可用good()函数查询是否成功
        _visualizationLCM(getLcmUrl(255)) {
    _controller = robot_ctrl;
    _userControlParameters = robot_ctrl->getUserControlParameters();
        }
  void prefaultStack();
  void setupScheduler();
  void initError(const char* reason, bool printErrno = false);
  void initCommon();
  ~WebotsBridge() { delete _robotRunner; }

  void handleControlParameter(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const control_parameter_request_lcmt* msg);
  void handleInterfaceLCM();
  void publishVisualizationLCM();

 protected:
  PeriodicTaskManager taskManager;                   //任务管理器
  Gamepad _gamepad;                                  //游戏手柄
  VisualizationData _visualizationData;              //可视化数据,调试用
  CheetahVisualization _mainCheetahVisualization;    //在仿真环境上绘制当前机器人?
  lcm::LCM _interfaceLCM;                            //lcm接口
  lcm::LCM _visualizationLCM;                        //用于可视化的lcm接口
  control_parameter_respones_lcmt _parameter_response_lcmt;  //控制参数响应lcm数据类型
  SpiData _spiData;                                  //spi过来的数据
  SpiCommand _spiCommand;                            //spi命令

  RobotRunner* _robotRunner = nullptr;               //机器人运行器,比较重要
  RobotControlParameters _robotParams;               //机器人控制参数
  std::thread _interfaceLcmThread;                   //lcm接口线程
  volatile bool _interfaceLcmQuit = false;     
  RobotController* _controller = nullptr;
  ControlParameters* _userControlParameters = nullptr;
};

/*!
 * Interface between robot and hardware specialized for Mini Cheetah
 * 专门用于mini cheetah的机器人与硬件之间的接口
 */
class MiniCheetahWebotsBridge : public WebotsBridge {
 public:
  MiniCheetahWebotsBridge(RobotController* rc);
  void run();
  void load_param();
  void abort(const std::string& reason);
  void abort(const char* reason);
  void handleWebots2ControllerLCM(const lcm::ReceiveBuffer* rbuf, 
                                  const std::string& chan,
                                  const webots2controller_t* msg);

 private:
  VectorNavData _vectorNavData;  //陀螺仪数据
  lcm::LCM webots2ControllerLCM;
  lcm::LCM controller2WebotsLCM;
};
#endif  // PROJECT_HARDWAREBRIDGE_H
