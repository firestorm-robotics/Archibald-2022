// Unified motor struct for Talons and Sparks.

#define MOTOR_TYPE_NONE  0
#define MOTOR_TYPE_TALON 1
#define MOTOR_TYPE_SPARK 2

struct Motor {
    TalonSRX* talon;
    rev::CANSparkMax* spark;
    uint8_t type = MOTOR_TYPE_NONE;

    Motor(TalonSRX controller) {
        talon = controller;
        type = MOTOR_TYPE_TALON;
    }

    Motor(rev::CANSparkMax controller) {
        spark = controller;
        type = MOTOR_TYPE_SPARK;
    }

    void _drivePercent(double amount) {
        if (type == MOTOR_TYPE_TALON) {
            talon -> Set(ControlMode::PercentOutput)
        }
    }

    void Set(uint8_t mode, double amount) {

    }

    void Set(uint8_t mode, long amount) { // These should handle both PID and normal drive.

    }
}
