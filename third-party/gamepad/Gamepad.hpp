/**
 * 
 */
#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <linux/joystick.h>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include "GamepadCommand.h"
class Gamepad
{
  public:
    enum EVENT_TYPE
    {
        AXIS = 2,
        BUTTON = 1
    };

    enum AXIS_ID
    {
        LEFT_X = 0,
        LEFT_Y = 1,
        LEFT_TRIGGER = 2,
        RIGHT_X = 3,
        RIGHT_Y = 4,
        RIGHT_TRIGGER = 5,
        PAD_X = 6,
        PAD_Y = 7
    };

    enum BUTTON_ID
    {
        A_BUTTON = 0,
        B_BUTTON = 1,
        X_BUTTON = 2,
        Y_BUTTON = 3,
        LEFT_BUMPER = 4,
        RIGHT_BUMPER = 5,
        BACK_BUTTON = 6,
        START_BUTTON = 7,
        LOGITECH_BUTTON = 8,
        LEFT_STICK_BUTTON = 9,
        RIGHT_STICK_BUTTON = 10
    };

  public:
    Gamepad();
    ~Gamepad();

    GamepadCommand get();
  private:
    bool Start();
    bool Read();
    void ThreadFunction();
    std::thread _thread;
    std::mutex _mutex;
    std::ifstream m_input;
    bool m_enabled;
    GamepadCommand cmd;
};

#endif // !GAMEPAD_H