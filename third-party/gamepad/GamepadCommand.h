/*! @file GamepadCommand.h
 *  @brief The GamepadCommand type containing joystick information
 */

#ifndef _GAMEPADCOMMAND_H
#define _GAMEPADCOMMAND_H

#include "Utilities/utilities.h"
#include "cppTypes.h"
/*!
 * The state of the gamepad
 * 此类用于描述游戏手柄的状态
 */
struct GamepadCommand {
  /*!
   * Construct a gamepad and set to zero.
   * 构建函数,并且设置参数为0
   */
  GamepadCommand() { zero(); }

  bool up, down, left, right, leftBumper, rightBumper, back,
      start, a, b, x, y, leftStickButton, rightStickButton, logitechButton;

  Vec2<float> leftStickAnalog, rightStickAnalog;
  float leftTriggerAnalog, rightTriggerAnalog;

  /*!
   * Set all values to zero
   * 设置所有参数为0
   */
  void zero() {
    up = false;
    down = false;
    left = false;
    right = false;
    leftBumper = false;
    rightBumper = false;
    back = false;
    start = false;
    a = false;
    b = false;
    x = false;
    y = false;
    leftStickButton = false;
    rightStickButton = false;
    logitechButton = false;
    leftTriggerAnalog = 0;
    rightTriggerAnalog = 0;
    leftStickAnalog = Vec2<float>::Zero();
    rightStickAnalog = Vec2<float>::Zero();
  }

  /*!
   * The Logitech F310's seem to do a bad job of returning to zero exactly, so a
   * deadband around zero is useful when integrating joystick commands
   * 罗技310手柄摇杆不能精确的归零,因此在0附近设置死区.
   * @param f : The deadband
   */
  void applyDeadband(float f) {
    eigenDeadband(leftStickAnalog, f);
    eigenDeadband(rightStickAnalog, f);
    leftTriggerAnalog = deadband(leftTriggerAnalog, f);
    rightTriggerAnalog = deadband(rightTriggerAnalog, f);
  }

  /*!
   * Represent as human-readable string.
   * 显示为人类可读的字符串
   * @return string representing state
   */
  std::string toString() {
    std::string result =
        "up: " + boolToString(up) + "\n" +
        "down: " + boolToString(down) + "\n" +
        "left: " + boolToString(left) + "\n" +
        "right: " + boolToString(right) + "\n" +
        "leftBumper: " + boolToString(leftBumper) + "\n" +
        "rightBumper: " + boolToString(rightBumper) + "\n" +
        "back: " + boolToString(back) + "\n" + "start: " + boolToString(start) +
        "\n" + "a: " + boolToString(a) + "\n" + "b: " + boolToString(b) + "\n" +
        "x: " + boolToString(x) + "\n" + "y: " + boolToString(y) + "\n" +
        "leftStickButton: " + boolToString(leftStickButton) + "\n" +
        "rightStickButton: " + boolToString(rightStickButton) + "\n" +
        "logitechButton: " + boolToString(logitechButton) + "\n" +
        "leftTriggerAnalog: " + std::to_string(leftTriggerAnalog) + "\n" +
        "rightTriggerAnalog: " + std::to_string(rightTriggerAnalog) + "\n" +
        "leftStickAnalog: " + eigenToString(leftStickAnalog) + "\n" +
        "rightStickAnalog: " + eigenToString(rightStickAnalog) + "\n";
    return result;
  }
};

#endif  // PROJECT_DRIVERCOMMAND_H
