// Unified motor struct for Talons and Sparks. Will have PID control.

#include <constants.h>
#include <util/functions.hpp>

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

    double PIDPos = 0;
    double PIDSpeed = 0;

    uint8_t opMode = MOTOR_PID_MODE_NONE;

    bool zero = false;

    bool locked = false; // Is it locked?
    void* lockObject = 0; // Don't let the motor be controlled by non-lockholders
    bool lockCommand = false; // Next command is issued by the lockholder

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
        util::setPIDPresets(ret -> sparkPID);
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

    void _driveCustonPIDTo(double pos, double reduction = 0.2){
        double dist = pos - GetPosition();
        double distPerc = util::sigmoid(10, dist);
        PIDPos = pos;
        _drivePercent(distPerc * reduction);
    }

    void _drivePIDSpeed(long speed){
        if (type == MOTOR_TYPE_TALON){

        }
        else if (type == MOTOR_TYPE_SPARK) {
            sparkPID.SetReference(speed, rev::CANSparkMax::ControlType::kVelocity);
        }
        PIDSpeed = speed;
    }

    void Set(double amount) {
        if (lockCommand || !locked){
            _drivePercent(amount);
            lockCommand = false;
        }
    }

    void SetPos(double pos) {
        //_drivePIDTo(pos);
        if (lockCommand || !locked){
            _driveCustonPIDTo(pos);
            lockCommand = false;
        }
    }

    double GetPosition(){
        if (type == MOTOR_TYPE_SPARK){
            return sparkEncoder.GetPosition();
        }
        return 0; // Just in case.
    }

    double GetVelocity(){
        if (type == MOTOR_TYPE_SPARK){
            return sparkEncoder.GetVelocity();
        }
        return 0;
    }

    void ZeroEncoder(){
        if (lockCommand || !locked){
            if (type == MOTOR_TYPE_SPARK){
                sparkEncoder.SetPosition(0);
            }
            zero = true;
        }
    }

    bool IsZeroed(){
        if (zero){
            if (fabs(GetPosition()) < 0.1){ // 0.1 encoder drift.
                zero = false;
                return true;
            }
            else{
                return false;
            }
        }
        return true;
    }

    bool SetSpeed(long speed){
        if (lockCommand || !locked){
            _drivePIDSpeed(speed);
            if (abs(GetVelocity() - speed) < PID_SPEED_ACCEPTABLE_ERROR) {
                return true;
            }
            lockCommand = false;
        }
        return false;
    }

    bool Lock(void* thing){
        if (!locked){
            locked = true;
            lockObject = thing;
            return true;
        }
        return false;
    }

    bool Unlock(void* thing){
        if (thing == lockObject && locked){
            locked = false;
            return true;
        }
        return false;
    }

    bool LockCommand(void* thing){
        if (thing == lockObject && locked){
            lockCommand = true;
            return true;
        }
        return false;
    }
};
#endif
