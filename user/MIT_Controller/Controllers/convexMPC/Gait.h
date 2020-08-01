#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H

#include <string>
#include <queue>

#include "cppTypes.h"


class Gait {
public:
  virtual ~Gait() = default;

  virtual Vec4<float> getContactState() = 0;
  virtual Vec4<float> getSwingState() = 0;
  virtual int* getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;
  virtual int getCurrentGaitPhase() = 0;
  virtual void debugPrint() { }

protected:
  std::string _name;
};

using Eigen::Array4f;
using Eigen::Array4i;

class OffsetDurationGait : public Gait {
public:
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~OffsetDurationGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  int* _mpc_table;
  Array4i _offsets;            // offset in mpc segments
  Array4i _durations;          // duration of step in mpc segments
  Array4f _offsetsFloat;       // offsets in phase (0 to 1)
  Array4f _durationsFloat;     // durations in phase (0 to 1)
  int _stance;                 // 以段为单位,支撑相时间.一段是两个MPC之间的时间间隔
  int _swing;                  // 以段为单位,摆动相时间
  int _iteration;              // 指在本迈步周期中,当前MPC是第几个
  int _nIterations;            // _swing+_stance,总段数
  float _phase;                // _phase是指目前迈步周期的完成度,(0%->100%)
};



class MixedFrequncyGait : public Gait {
public:
  MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string& name);
  ~MixedFrequncyGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  float _duty_cycle;
  int* _mpc_table;
  Array4i _periods;
  Array4f _phase;
  int _iteration;
  int _nIterations;
};

#endif //PROJECT_GAIT_H
