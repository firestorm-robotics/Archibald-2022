// Unified motor struct for Talons and Sparks. Will have PID control.

#ifndef Motor_hpp_INCLUDED // If it hasn't been included in this project before, pass this line.
#define Motor_hpp_INCLUDED // Flag it as included so next time somebody does #include <util/Motor.hpp>, it uses the old one instead.

#define MOTOR_TYPE_NONE  0
#define MOTOR_TYPE_TALON 1
#define MOTOR_TYPE_SPARK 2

#define MOTOR_PID_MODE_NONE  0
#define MOTOR_PID_MODE_SPEED 1
#define MOTOR_PID_MODE_POS   2

struct Motor {
    TalonSRX* talon;
    rev::CANSparkMax* spark;
    rev::SparkMaxPIDController sparkPID;
    rev::SparkMaxRelativeEncoder sparkEncoder;
    uint8_t type = MOTOR_TYPE_NONE;

    long long PIDPos = 0;
    long long PIDSpeed = 0;

    uint8_t opMode = MOTOR_PID_MODE_NONE;

    static Motor* CreateMotor(TalonSRX* controller) {
        Motor* ret = (Motor*)malloc(sizeof(Motor));
        memset(ret, 0, sizeof(Motor));
        ret -> talon = controller;
        ret -> type = MOTOR_TYPE_TALON;
        return ret;
    }

    static Motor* CreateMotor(rev::CANSparkMax* controller) {
        Motor* ret = (Motor*)malloc(sizeof(Motor));
        memset(ret, 0, sizeof(Motor));
        ret -> spark = controller;
        ret -> type = MOTOR_TYPE_SPARK;
        ret -> sparkPID = controller -> GetPIDController();
        ret -> sparkEncoder = controller -> GetEncoder();
        return ret;
    }

    void _drivePercent(double amount) {
        if (type == MOTOR_TYPE_TALON) {
            talon -> Set(ControlMode::PercentOutput, amount);
        }
        else if (type == MOTOR_TYPE_SPARK){
            spark -> Set(amount);
        }
    }

    void _drivePIDTo(long pos){
        if (type == MOTOR_TYPE_TALON){

        }
        else if (type == MOTOR_TYPE_SPARK){
            sparkPID.SetReference(pos, rev::CANSparkMax::ControlType::kPosition);
        }
        PIDPos = pos;
    }

    void _drivePIDSpeed(long speed){
        if (type == MOTOR_TYPE_TALON){

        }
        else {
            sparkPID.SetReference(speed, rev::CANSparkMax::ControlType::kVelocity);
        }
        PIDSpeed = speed;
    }

    void Set(double amount) {
        _drivePercent(amount);
    }

    void SetPos(long pos) {
        _drivePIDTo(pos);
    }

    bool checkDone(){
        if (type == MOTOR_TYPE_SPARK){
            if (opMode == MOTOR_PID_MODE_POS){
                if (sparkEncoder.GetPosition() == PIDPos){
                    return true;
                }
                else {
                    return false;
                }
            }
            else if (opMode == MOTOR_PID_MODE_SPEED){
                if (sparkEncoder.GetVelocity() == PIDSpeed){
                    return true;
                }
                else {
                    return false;
                }
            }
        }
        return true; // If there is no PID opmode set and/or no motor type set, this returns true for safety. This statement will only ever evaluate if one of those conditions is true.
    }
};
#endif
