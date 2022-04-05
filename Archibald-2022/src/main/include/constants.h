// Constants for the entire project.
/* Check Robot_Graphic.txt for CAN id graphics. */

// Constants for motors
#define MOTOR_FRONT_LEFT       1
#define MOTOR_FRONT_RIGHT      2
#define MOTOR_BACK_LEFT        3
#define MOTOR_BACK_RIGHT       4
#define MOTOR_SHOOTER_LEFT     6 // 5
#define MOTOR_SHOOTER_RIGHT    5 // 6
#define MOTOR_INDEXER          7
#define MOTOR_INTAKE           8
#define MOTOR_INTAKE_DROP      9
#define MOTOR_CLIMB            10
#define MOTOR_STATICHOOK_LEFT  11
#define MOTOR_STATICHOOK_RIGHT 12

#define NEO_500_RPM        5676

#define PID_ACCEPTABLE_ERROR 100

#define UNIVERSAL_DEADBAND 0.1 // The universal deadband for controls, this helps with our high-sensitivity issue.


const double PID_kP = 0.00001/*0.00004*/, PID_kI = 0, PID_kD = 0/*0.00025*/, PID_kIz = 0, PID_kFF = /*0*/0.000175, PID_kMaxOutput = 1, PID_kMinOutput = -1;

#define BUTTONBOARD_PANIC                 8
#define BUTTONBOARD_DROP_INTAKE           3
#define BUTTONBOARD_RAISE_INTAKE          4
#define BUTTONBOARD_MACRO_INTAKE          5
#define BUTTONBOARD_INDEXER_UP            11
#define BUTTONBOARD_INDEXER_DOWN          13
#define BUTTONBOARD_SHOOTERSPEED_LOW      6
#define BUTTONBOARD_SHOOTERSPEED_VARIABLE 10
#define BUTTONBOARD_SHOOTER_AXIS          2
#define BUTTONBOARD_SHOOTER               12
#define BUTTONBOARD_SHOOTER_MACRO         9

#define XBOX_BUMPER_LEFT  5
#define XBOX_BUMPER_RIGHT 10
#define XBOX_DPAD_UP      -1
#define XBOX_DPAD_DOWN    -1

#define SHOOTERSPEED_LOW  NEO_500_RPM * 0.4
#define SHOOTERSPEED_MID  NEO_500_RPM * 0.73
#define SHOOTERSPEED_HIGH NEO_500_RPM * 0.7

#define JOYSTICK_INDEXER_UP   6
#define JOYSTICK_INDEXER_DOWN 4

#define CLIMBERSWITCH 1

#define LIMELIGHT_HUB_ANGLE_FOR_SHOOTING 21

#define AUTONOMOUS_ROTS_GOALTOBALL 100
#define AUTONOMOUS_ANGLE_TOBALL    30

#define PHOTOELECTRIC_SHOOTER 7
#define PHOTOELECTRIC_INTAKE  8
