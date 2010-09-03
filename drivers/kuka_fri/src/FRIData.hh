#ifndef FRIDATA_HH
#define FRIDATA_HH

class RobotStatus
{
public:
  RobotStatus();

  //TODO: add uptime information

  // connection statistics
  float time;        //!< robot internal time
  float answerRate;  //!< average answer rate
  float latency;     //!< average latency, including KUKA internal delays
  float jitter;      //!< average jitter
  float missRate;    //!< rate of missing packets
  int missCounter; //!< number of missed answer packets

  // interface info
  int state; //!< state of the interface (1 = monitor 2 = command)
  int quality; //!< communication quality (0 = unacceptable, 1 = bad, 2 = okay, 3 = perfect)
  float msrSampleTime; //!< sample time for measurement packets
  float cmdSampleTime; //!< sample time for command packets
  float safety; //!< scaling factor for safety limits [0..1]

  // general robot info
  int power; //!< power state of the motors (bitfield)
  int controlMode; //!< control mode
  int error; //!< error code
  int warning; //!< warning code
  float temperature[7]; //!< temperatures of the motors

  // driver stuff
  bool runstop;
};

//! measured data from robot
class RobotData
{
public:
  RobotData();
  int seq;
  float commanded[7];
  float position[7];
  float torque[7];
  float torqueTCP[6];
};


//! command data for robot (to be executed within the next cycle)
class RobotCommand
{
  static const float DEFAULT_STIFFNESS=40;
  static const float DEFAULT_DAMPING=0.7;
public:
  RobotCommand();
  void incrementPosition(float* newPos, float* currentPos, float rate); //!< increment
  float command[7];   //!< desired joint angle CHANGE [rad/s] (relative to current joint position)
  float stiffness[7]; //!< stiffness [Nm/rad]
  float damping[7];   //!< damping [normaized 0..1]
  float addTorque[7]; //!< additional torque [Nm]
};

#endif // FRIDATA_HH