// Constants for the entire project.
/* Check Robot_Graphic.txt for CAN id graphics. */

// Constants for motors

#ifndef CONSTANTS_H // Make sure it only really #includes once.
#define CONSTANTS_H

#define NEO_500_RPM        5676

#define PID_SPEED_ACCEPTABLE_ERROR 100
#define PID_POSITION_ACCEPTABLE_ERROR 1

#define UNIVERSAL_DEADBAND 0.1 // The universal deadband for controls, this helps with our high-sensitivity issue.


const double PID_kP = 0.00001/*0.00004*/, PID_kI = 0, PID_kD = 0/*0.00025*/, PID_kIz = 0, PID_kFF = /*0*/0.000175, PID_kMaxOutput = 1, PID_kMinOutput = -1;

#define SHOOTERSPEED_LOW  NEO_500_RPM * 0.4
#define SHOOTERSPEED_MID  NEO_500_RPM * 0.75
#define SHOOTERSPEED_HIGH NEO_500_RPM * 0.7


#define CLIMBERSWITCH 1

#define LIMELIGHT_HUB_ANGLE_FOR_SHOOTING 21

#define AUTO_MIDSTALK_LENGTH 34

#include <CANids.h>
#include <controllerNumbers.h>
#include <sensorPins.h>

#endif
