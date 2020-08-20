#include "Gamepad.hpp"
#include <linux/input.h>
#include <linux/joystick.h>
#include <iostream>
#include <stdint.h>
#include <sstream>
#include <unistd.h>
#include <thread>
#include <mutex>
#include "GamepadCommand.h"


/** Default files for controller input in linux evdev and js and js in xinput mode **/
const std::string f710File_evdev = "/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick";
const std::string f710File_js = "/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-joystick";
const std::string f710File_js_xmode = "/dev/input/js0";

Gamepad::Gamepad() : m_enabled(false)
{
    cmd.zero();
    _thread = std::thread(&Gamepad::ThreadFunction, this);
    printf("[GAMEPAD] Instantiated, thread start. \n");
}

Gamepad::~Gamepad()
{
    m_input.close();
    _thread.join();
}
GamepadCommand Gamepad::get()
{
    static GamepadCommand new_cmd;
    _mutex.lock();
    memcpy(&new_cmd,&cmd,sizeof(GamepadCommand));
    _mutex.unlock();
    return new_cmd;
}

void Gamepad::ThreadFunction()
{
    this->Start();
    for(;;)
    {
        usleep(2000); //两次连续数据读取时间间隔不超过2ms
        this->Read();
    }
}

bool Gamepad::Start()
{
    static int fail_iter = 0;
    // Reset controller if necessary.
    if (m_input.is_open()) m_input.close();
    // Open controller in x-input mode.
    m_input.open(f710File_js_xmode.c_str(), std::ifstream::binary | std::ifstream::in);
    // If controller failed to open, set our status to false.
    if (!m_input.is_open())
    {
        printf("[GAMEPAD] try start but faild：%d\n", fail_iter++);
        m_enabled = false;
        return false;
    }
    printf("[GAMEPAD] start success!\n");
    m_enabled = true;
    fail_iter = 0;
    return true;
}

bool Gamepad::Read()
{
    //重新启动
    if (!m_enabled)
    {
        usleep(500000);    //每隔 0.5s尝试一次新的打开
        if(!Start()) return false;
    }
    //读数据
    js_event event;
    m_input.read((char *)&event, sizeof(struct js_event));
    // Check to make sure we got a full read.
    if (m_input.gcount() != sizeof(struct js_event))
    {
        printf("[GAMEPAD] read nothing, please check whether the handle is pulled out!\n");
        m_enabled = false;
        return false;
    }
    //处理数据
    _mutex.lock();
    if(event.type == Gamepad::EVENT_TYPE::AXIS)
    {
        float value = (((double)event.value / ((double)INT16_MAX)));
        switch (event.number)
        {
            case LEFT_X        : cmd.leftStickAnalog[0]  =  value; break;
            case LEFT_Y        : cmd.leftStickAnalog[1]  = -value; break;
            case LEFT_TRIGGER  : cmd.leftTriggerAnalog   =  (value+1)/2; break;
            case RIGHT_X       : cmd.rightStickAnalog[0] =  value; break;
            case RIGHT_Y       : cmd.rightStickAnalog[1] = -value; break;
            case RIGHT_TRIGGER : cmd.rightTriggerAnalog  =  (value+1)/2; break;
            case PAD_X:
                cmd.right = value > 0;
                cmd.left  = value < 0; break;
            case PAD_Y:
                cmd.down = value > 0;
                cmd.up   = value < 0; break;
            default: break;
        }
    }
    else if(event.type == Gamepad::EVENT_TYPE::BUTTON)
    {
        bool value = (event.value == 1);
        switch (event.number)
        {
            case A_BUTTON           : cmd.a                = value; break;
            case B_BUTTON           : cmd.b                = value; break;
            case X_BUTTON           : cmd.x                = value; break;
            case Y_BUTTON           : cmd.y                = value; break;
            case LEFT_BUMPER        : cmd.leftBumper       = value; break;
            case RIGHT_BUMPER       : cmd.rightBumper      = value; break;
            case BACK_BUTTON        : cmd.back             = value; break;
            case START_BUTTON       : cmd.start            = value; break;
            case LOGITECH_BUTTON    : cmd.logitechButton   = value; break;
            case LEFT_STICK_BUTTON  : cmd.leftStickButton  = value; break;
            case RIGHT_STICK_BUTTON : cmd.rightStickButton = value; break;
            default: break;
        }

        static bool pre_logitech_value = false;
        if(event.number == LOGITECH_BUTTON)
        {
            if((pre_logitech_value == false)&&(value == true))
            {
                printf("\n[GAMEPAD] I am alive, Here's my data:\n");
                printf("%s\n", cmd.toString().c_str());
            }
            pre_logitech_value  = value;
        }
    }
    cmd.applyDeadband(0.05);
    _mutex.unlock();
    return true;
}