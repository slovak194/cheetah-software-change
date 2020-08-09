#include "Gait.h"

// Offset - Duration Gait   偏移-占空比 步态,通过偏移量与占空比指定
OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string &name) :
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nSegment)
{
  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];

  _offsetsFloat = offsets.cast<float>() / (float) nSegment;
  _durationsFloat = durations.cast<float>() / (float) nSegment;

  _stance = durations[0];             //支撑相时间
  _swing = nSegment - durations[0];   //摆动相时间,单位为mpc段
}

OffsetDurationGait::~OffsetDurationGait() {
  delete[] _mpc_table;
}

//用来干什么的?
Vec4<float> OffsetDurationGait::getContactState() {
  Array4f progress = _phase - _offsetsFloat;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _durationsFloat[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

Vec4<float> OffsetDurationGait::getSwingState()
{
  Array4f swing_offset = _offsetsFloat + _durationsFloat;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  Array4f swing_duration = 1. - _durationsFloat;

  Array4f progress = _phase - swing_offset;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}


int* OffsetDurationGait::getMpcTable()
{
  //printf("MPC table:\n");
  for(int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])
        _mpc_table[i*4 + j] = 1;     //支撑相是1
      else
        _mpc_table[i*4 + j] = 0;     //摆动相是0

      //printf("%d ", _mpc_table[i*4 + j]);
    }
    //printf("\n");
  }
  return _mpc_table;
}
//iterationsPerMPC是两个MPC之间的WBC迭代次数.
//currentIteration是wbc的总迭代次数
//_nIterations是指在一个迈步周期中,会有多少个MPC
//_iteration是指在本迈步周期中,当前MPC是第几个
//_phase是指目前迈步周期的完成度,(0%->100%)
void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);
}

bool OffsetDurationGait::isGaitEnd()
{
  if(_phase > 0.95) return true;
  else return false;
}

float OffsetDurationGait::getCurrentSwingTime(float dtMPC) {
  return dtMPC * _swing;
}

float OffsetDurationGait::getCurrentStanceTime(float dtMPC) {
  return dtMPC * _stance;
}

float OffsetDurationGait::getCurrentGaitTime(float dtMPC) {
  return dtMPC * _nIterations;
}

void OffsetDurationGait::debugPrint() {

}