// Constants for the entire project.
/* Check Robot_Graphic.txt for CAN id graphics. */

// Constants for motors
#define MOTOR_FRONT_LEFT      1 /* This is the front left motor, check the graphic. */
#define MOTOR_FRONT_RIGHT     2
#define MOTOR_BACK_LEFT       3
#define MOTOR_BACK_RIGHT      4
#define MOTOR_SHOOTER_LEFT    6 // 5
#define MOTOR_SHOOTER_RIGHT   5 // 6
#define MOTOR_INDEXER         7
#define MOTOR_INTAKE          8
#define MOTOR_INTAKE_DROP     9

#define NEO_500_RPM        5676

#define UNIVERSAL_DEADBAND 0.1 // The universal deadband for controls, this helps with our high-sensitivity issue.

#define ENCODER_TICKS_PER_INCH 647 // See distance-math.txt in the root of this project.

#define PID_ENCODER_ERROR 50 // Acceptable error rate for a PID encoder.


const double PID_kP = 0.1, PID_kI = 1e-4, PID_kD = 1, PID_kIz = 0, PID_kFF = 0, PID_kMaxOutput = 0.5, PID_kMinOutput = -0.5;
