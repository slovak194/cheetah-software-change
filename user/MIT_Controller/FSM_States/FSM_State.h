#ifndef FSM_State_H
#define FSM_State_H

#include <stdio.h>

#include "ControlFSMData.h"
#include "TransitionData.h"

// Normal robot states
#define K_INVALID         (-1)
#define K_PASSIVE         (0)
#define K_SIT_DOWN        (1)
#define K_BALANCE_STAND   (3)
#define K_LOCOMOTION      (4)
#define K_RECOVERY_STAND  (6)

/**
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum class FSM_StateName {
  INVALID,
  PASSIVE,
  SIT_DOWN,
  BALANCE_STAND,
  LOCOMOTION,
  RECOVERY_STAND
};

/**
 *
 */
template <typename T>
class FSM_State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Generic constructor for all states
  FSM_State(ControlFSMData<T>* _controlFSMData, FSM_StateName stateNameIn,
            std::string stateStringIn);

  // Behavior to be carried out when entering a state
  virtual void onEnter() = 0;// {}

  // Run the normal behavior for the state
  virtual void run() = 0; //{}

  // Manages state specific transitions
  virtual FSM_StateName checkTransition() { return FSM_StateName::INVALID; }

  // Runs the transition behaviors and returns true when done transitioning
  virtual TransitionData<T> transition() { return transitionData; }

  // Behavior to be carried out when exiting a state
  virtual void onExit() = 0; // {}

  //
  void jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes);
  void cartesianImpedanceControl(int leg, Vec3<T> pDes, Vec3<T> vDes,
                                 Vec3<double> kp_cartesian,
                                 Vec3<double> kd_cartesian);
  void footstepHeuristicPlacement(int leg);

  //
  void turnOnAllSafetyChecks();
  void turnOffAllSafetyChecks();

  // Holds all of the relevant control data
  ControlFSMData<T>* _data;

  // FSM State info
  FSM_StateName stateName;      // enumerated name of the current state
  FSM_StateName nextStateName;  // enumerated name of the next state
  std::string stateString;      // state name string

  // Transition parameters
  T transitionDuration;  // transition duration time
  T tStartTransition;    // time transition starts
  TransitionData<T> transitionData;

  // Pre controls safety checks
  bool checkSafeOrientation = false;  // check roll and pitch

  // Post control safety checks
  bool checkPDesFoot = false;          // do not command footsetps too far
  bool checkForceFeedForward = false;  // do not command huge forces
  bool checkLegSingularity = false;    // do not let leg

  // Leg controller command placeholders for the whole robot (3x4 matrices)
  Mat34<T> jointFeedForwardTorques;  // feed forward joint torques
  Mat34<T> jointPositions;           // joint angle positions
  Mat34<T> jointVelocities;          // joint angular velocities
  Mat34<T> footFeedForwardForces;    // feedforward forces at the feet
  Mat34<T> footPositions;            // cartesian foot positions
  Mat34<T> footVelocities;           // cartesian foot velocities

  // Footstep locations for next step
  Mat34<T> footstepLocations;

 private:
  // Create the cartesian P gain matrix
  Mat3<float> kpMat;

  // Create the cartesian D gain matrix
  Mat3<float> kdMat;
};

#endif  // FSM_State_H
