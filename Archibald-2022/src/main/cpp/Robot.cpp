#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>
#include "ModularRobot.hpp"
#include "rev/CANSparkMax.h"
#include <constants.h>
#include <frc/Joystick.h>
#include <frc/DigitalInput.h>
#include <cmath>

class Robot : public ModularRobot{
public:
    rev::CANSparkMax frontLeft{MOTOR_FRONT_LEFT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax frontRight{MOTOR_FRONT_RIGHT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax backLeft{MOTOR_BACK_LEFT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax backRight{MOTOR_BACK_RIGHT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax indexer{MOTOR_INDEXER, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterRight{MOTOR_SHOOTER_RIGHT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterLeft{MOTOR_SHOOTER_LEFT, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder shooterRight_encoder = shooterRight.GetEncoder();

    frc::Joystick controls{5};
    frc::DigitalInput test{2};
    Robot(){
        setData("Archibald", "Firestorm Robotics", 6341);
    }

    void Init(){

    }

    bool runningShooter = false;
    bool triggerClick = false;

    bool runningIndexer = false;
    bool indexerButtonClick = false;

    uint8_t motorMode = 0; // 0 = max speed, 1 = variable mode
    static uint8_t maxMode = 1;
    bool modeButtonClick = false;

    void TeleopLoop(){
        double limit = (controls.GetThrottle() + 1) / 2;
        double forThrust = controls.GetY() * limit;
        double sideThrust = controls.GetX() * limit;
        if (!(runningIndexer && abs(controls.GetTwist()) > 0.05)){
            frontLeft.Set(forThrust + sideThrust);
            backLeft.Set(forThrust + sideThrust);
            frontRight.Set(-forThrust + sideThrust);
            frontRight.Set(-forThrust + sideThrust);
        }
        if (controls.GetTrigger()){
            triggerClick = true;
        }
        else if (triggerClick){
            runningShooter = !runningShooter;
            triggerClick = false;
        }
        if (runningShooter){
            if (motorMode == 0){
                shooterRight.Set(1);
                shooterLeft.Set(-1);
            }
            else if (motorMode == 1){
                shooterRight.Set(limit * 2);
                shooterLeft.Set(limit * -2);
            }
        }
        else{
            shooterRight.Set(0);
            shooterLeft.Set(0);
        }
        if (controls.GetRawButton(2)){
            indexerButtonClick = true;
        }
        else if (indexerButtonClick){
            indexerButtonClick = false;
            runningIndexer = !runningIndexer;
        }
        if (runningIndexer){
            indexer.Set(controls.GetTwist() * limit);
        }
        else{
            indexer.Set(0);
        }
        if (controls.GetRawButton(3)){
            modeButtonClick = true;
        }
        else if (modeButtonClick){
            motorMode ++;
            if (motorMode > maxMode){
                motorMode = 0;
            }
            modeButtonClick = false;
        }
        if (test.Get() == 0){
            printf("You Win!");
        }
    }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
