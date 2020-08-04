/*!
 * @file WebotsBridge.cpp
 * @brief Interface between robot code and webots simulator
 * @brief 阻塞式等待webots发来的信息,经控制器计算后再发回去.
 */
#ifdef linux

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include "Configuration.h"
#include "orientation_tools.h"
#include "WebotsBridge.hpp"
#include "Utilities/Utilities_print.h"
#include "webots2controller_t.hpp"
#include "controller2webots_t.hpp"

/*!
 * If an error occurs during initialization, before motors are enabled, print
 * error and exit.
 * @param reason Error message string
 * @param printErrno If true, also print C errno
 */
void WebotsBridge::initError(const char* reason, bool printErrno) {
  printf("FAILED TO INITIALIZE HARDWARE: %s\n", reason);

  if (printErrno) {
    printf("Error: %s\n", strerror(errno));
  }

  exit(-1);
}

/*!
 * All hardware initialization steps that are common between Cheetah 3 and Mini Cheetah
 * 所有猎豹3与猎豹mini公共的硬件初始化步骤
 */
void WebotsBridge::initCommon() {
  printf("[WebotsBridge] Init stack\n");
  prefaultStack();     //初始化一个16k的堆栈缓存
  printf("[WebotsBridge] Init scheduler\n");
  setupScheduler();    //给当前进程设置调度策略与调度参数
  if (!_interfaceLCM.good()) {  //检查lcm是否初始化成功
    initError("_interfaceLCM failed to initialize\n", false);
  }

  printf("[WebotsBridge] Subscribe LCM\n");
  // 订阅手柄LCM消息,用于接受手柄信息
  _interfaceLCM.subscribe("interface", &WebotsBridge::handleGamepadLCM, this);
  // 订阅控制参数LCM消息,包括机器人参数与用户参数
  _interfaceLCM.subscribe("interface_request",
                          &WebotsBridge::handleControlParameter, this);

  printf("[WebotsBridge] Start interface LCM handler\n");
  // 这个线程用来运行handle函数
  _interfaceLcmThread = std::thread(&WebotsBridge::handleInterfaceLCM, this);
}
/*!
 * Run interface LCM
 */
void WebotsBridge::handleInterfaceLCM() {
  while (!_interfaceLcmQuit) _interfaceLCM.handle();
}

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 * stack, this will make sure that we won't have a page fault when the stack
 * grows.  Also mlock's all pages associated with the current process, which
 * prevents the cheetah software from being swapped out.  If we do run out of
 * memory, the robot program will be killed by the OOM process killer (and
 * leaves a log) instead of just becoming unresponsive.
 * 写入到堆栈上的一个16 KB的缓冲区。如果我们为堆栈使用4K页面，这将确保在堆栈增长时不会出现页面错误。
 * 还有mlock与当前进程关联的所有页面，这可以防止cheetah软件被换出。
 * 如果内存耗尽，机器人程序将被OOM进程杀手杀死(并留下日志)，而不是变得没有响应。
 * 注意,可能需要root权限
 */
void WebotsBridge::prefaultStack() {
  printf("[Init] Prefault stack...\n");
  volatile char stack[MAX_STACK_SIZE];     //MAX_STACK_SIZE=16384 16kb
  memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    initError(
        "mlockall failed.  This is likely because you didn't run robot as "
        "root.\n",
        true);
  }
}

/*!
 * Configures the scheduler for real time priority
 * 如果pid为零，将会为调用进程设置调度策略和调度参数。
 * 如果进程pid含多个进程或轻量进程(即该进程是多进程的)，此函数将影响进程中各个子进程。
 */
void WebotsBridge::setupScheduler() {
  printf("[Init] Setup RT Scheduler...\n");
  struct sched_param params;
  params.sched_priority = TASK_PRIORITY;  //TASK_PRIORITY  = 49
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {  
    initError("sched_setscheduler failed.\n", true);
  }
}

/*!
 * LCM Handler for gamepad message
 * 游戏手柄的LCM回调函数
 * 在这里,从外部某个地方传来手柄信息,lcm消息由sim面板发送
 */
void WebotsBridge::handleGamepadLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const gamepad_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  _gamepadCommand.set(msg);
}
/*!
 * LCM Handler for control parameters
 * 控制参数的LCM回调函数
 */
void WebotsBridge::handleControlParameter(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const control_parameter_request_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  if (msg->requestNumber <= _parameter_response_lcmt.requestNumber) {
    // nothing to do!
    //判断消息号是否大于响应号,消息从先到后应该有一个独一无二的消息号
    printf(
        "[WebotsBridge] Warning: the interface has run a ControlParameter "
        "iteration, but there is no new request!\n");
    // return;
  }

  // sanity check
  // 完整性检查
  s64 nRequests = msg->requestNumber - _parameter_response_lcmt.requestNumber;
  //nRequests>1,说明有nRequests-1个消息未响应
  if (nRequests != 1) {
    printf("[ERROR] Hardware bridge: we've missed %ld requests\n",
           nRequests - 1);
  }

  switch (msg->requestKind) {
    case (s8)ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
      //没有使用用户参数
      if(!_userControlParameters) {
        printf("[Warning] Got user param %s, but not using user parameters!\n",
               (char*)msg->name);
      } else {
        std::string name((char*)msg->name);
        ControlParameter& param = _userControlParameters->collection.lookup(name);

        // type check
        // 类型检查
        if ((s8)param._kind != msg->parameterKind) {
          throw std::runtime_error(
              "type mismatch for parameter " + name + ", robot thinks it is " +
              controlParameterValueKindToString(param._kind) +
              " but received a command to set it to " +
              controlParameterValueKindToString(
                  (ControlParameterValueKind)msg->parameterKind));
        }

        // do the actual set
        // 进行赋值
        ControlParameterValue v;
        memcpy(&v, msg->value, sizeof(v));
        param.set(v, (ControlParameterValueKind)msg->parameterKind);

        // respond:
        // 回复,_parameter_response_lcmt赋值,在switch语句最后发送出去
        _parameter_response_lcmt.requestNumber =
            msg->requestNumber;  // acknowledge that the set has happened
        _parameter_response_lcmt.parameterKind =
            msg->parameterKind;  // just for debugging print statements
        memcpy(_parameter_response_lcmt.value, msg->value, 64);
        //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
        //for debugging print statements
        strcpy((char*)_parameter_response_lcmt.name,
               name.c_str());  // just for debugging print statements
        _parameter_response_lcmt.requestKind = msg->requestKind;

        printf("[User Control Parameter] set %s to %s\n", name.c_str(),
               controlParameterValueToString(
                   v, (ControlParameterValueKind)msg->parameterKind)
                   .c_str());
      }
    } break;

    case (s8)ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {
      std::string name((char*)msg->name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if ((s8)param._kind != msg->parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(
                (ControlParameterValueKind)msg->parameterKind));
      }

      // do the actual set
      ControlParameterValue v;
      memcpy(&v, msg->value, sizeof(v));
      param.set(v, (ControlParameterValueKind)msg->parameterKind);

      // respond:
      _parameter_response_lcmt.requestNumber =
          msg->requestNumber;  // acknowledge that the set has happened
      _parameter_response_lcmt.parameterKind =
          msg->parameterKind;  // just for debugging print statements
      memcpy(_parameter_response_lcmt.value, msg->value, 64);
      //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
      //for debugging print statements
      strcpy((char*)_parameter_response_lcmt.name,
             name.c_str());  // just for debugging print statements
      _parameter_response_lcmt.requestKind = msg->requestKind;

      printf("[Robot Control Parameter] set %s to %s\n", name.c_str(),
             controlParameterValueToString(
                 v, (ControlParameterValueKind)msg->parameterKind)
                 .c_str());

    } break;

    default: {
      throw std::runtime_error("parameter type unsupported");
    }
    break;
  }
  // 返回操作完成
  _interfaceLCM.publish("interface_response", &_parameter_response_lcmt);
}

//MiniCheetahWebotsBridge的构造函数
MiniCheetahWebotsBridge::MiniCheetahWebotsBridge(RobotController* robot_ctrl)
    : WebotsBridge(robot_ctrl), webots2ControllerLCM(getLcmUrl(255)) ,controller2WebotsLCM(getLcmUrl(255)){
  //nothing
}
/*!
 * 处理从webots传递来的数据
 */
void MiniCheetahWebotsBridge::handleWebots2ControllerLCM(const lcm::ReceiveBuffer* rbuf,
                                                         const std::string& chan,
                                                         const webots2controller_t* msg) {
  (void)rbuf;
  (void)chan;

  for(int leg=0;leg<4;leg++)
  {
    _spiData.q_abad[leg] = msg->q_abad[leg];
    _spiData.q_hip[leg] = msg->q_hip[leg];
    _spiData.q_knee[leg] = msg->q_knee[leg];
    _spiData.qd_abad[leg] = msg->qd_abad[leg];
    _spiData.qd_hip[leg] = msg->qd_hip[leg];
    _spiData.qd_knee[leg] = msg->qd_knee[leg];
  }
  _vectorNavData.accelerometer << msg->acc[0],msg->acc[1],msg->acc[2];
  _vectorNavData.gyro << msg->gyro[0],msg->gyro[1],msg->gyro[2];
  Vec3<float> RPY(msg->rpy[0], msg->rpy[1], msg->rpy[2]);
  Quat<float> q_wxyz = rpyToQuat(RPY);
  //xyzw = wxyz
  _vectorNavData.quat[0] = q_wxyz[1];   //x
  _vectorNavData.quat[1] = q_wxyz[2];   //y
  _vectorNavData.quat[2] = q_wxyz[3];   //z
  _vectorNavData.quat[3] = q_wxyz[0];   //w
}
/*!
 * 加载机器人参数与用户参数
 */
void MiniCheetahWebotsBridge::load_param() {
    //加载机器人参数
  printf("[Webots Bridge] Loading parameters from file...\n");

  try {
    _robotParams.initializeFromYamlFile(THIS_COM "config/mini-cheetah-defaults.yaml");
  } catch(std::exception& e) {
    printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
    exit(1);
  }

  if(!_robotParams.isFullyInitialized()) {
    printf("Failed to initialize all robot parameters\n");
    exit(1);
  }

  printf("Loaded robot parameters\n");
  //加载用户参数
  if(_userControlParameters) {
    try {
      if(this->_controller->ctrl_name == "mit_ctrl")
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters.yaml");
      else if(this->_controller->ctrl_name == "jpos_ctrl")
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/c3-jpos-user-parameters.yaml");
    } catch(std::exception& e) {
      printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
      exit(1);
    }

    if(!_userControlParameters->isFullyInitialized()) {
      printf("Failed to initialize all user parameters\n");
      exit(1);
    }

    printf("Loaded user parameters\n");
  } else {
    printf("Did not load user parameters because there aren't any\n");
  }

  printf("[Webots Bridge] Got all parameters, starting up!\n");
}
/*!
 * Main method for Mini Cheetah hardware
 * mini猎豹硬件主程序
 */
void MiniCheetahWebotsBridge::run() {
  initCommon();                      //初始化猎豹3和猎豹mini共有的参数
  _vectorNavData.quat << 1, 0, 0, 0; //初始化四元数,防止计算
  
  load_param();

  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

  _robotRunner->driverCommand = &_gamepadCommand; //手柄数据
  _robotRunner->spiData = &_spiData;              //spiData指针
  _robotRunner->spiCommand = &_spiCommand;        //spiCommand命令
  _robotRunner->vectorNavData = &_vectorNavData;  //陀螺仪
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->visualizationData = &_visualizationData;
  _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;
  _robotRunner->init();

  // visualization start   这个可以暂时保留,看看有什么效果
  PeriodicMemberFunction<MiniCheetahWebotsBridge> visualizationLCMTask(
      &taskManager, .0167, "lcm-vis",
      &MiniCheetahWebotsBridge::publishVisualizationLCM, this);
  visualizationLCMTask.start();

  // 订阅手柄LCM消息,用于接受webots传递来的信息
  printf("webots2Controller subscribe\r\n");
  webots2ControllerLCM.subscribe("webots2Controller", &MiniCheetahWebotsBridge::handleWebots2ControllerLCM, this);

  for (;;) {
    webots2ControllerLCM.handle();//阻塞等待
    //此时,webots数据已经传入了spidata,vectorNavData
     _robotRunner->run();
    //此时_spiCommand已经存入了controller计算的数据
    controller2webots_t c2w;
    memcpy(&c2w, &_spiCommand, sizeof(controller2webots_t));
    controller2WebotsLCM.publish("controller2Webots",&c2w);
  }
}
/*!
 * Send LCM visualization data
 */
void WebotsBridge::publishVisualizationLCM() {
  cheetah_visualization_lcmt visualization_data;
  //位置信息
  for (int i = 0; i < 3; i++) {
    visualization_data.x[i] = _mainCheetahVisualization.p[i];
  }
  //颜色与四元素?
  for (int i = 0; i < 4; i++) {
    visualization_data.quat[i] = _mainCheetahVisualization.quat[i];
    visualization_data.rgba[i] = _mainCheetahVisualization.color[i];
  }
  //关节信息
  for (int i = 0; i < 12; i++) {
    visualization_data.q[i] = _mainCheetahVisualization.q[i];
  }

  _visualizationLCM.publish("main_cheetah_visualization", &visualization_data);
}

#endif
