#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>
#include "ModularRobot.hpp"
#include "rev/CANSparkMax.h"
#include <constants.h>
#include <frc/Joystick.h>

class Robot : public ModularRobot{
public:
    rev::CANSparkMax frontLeft{1, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax frontRight{2, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax backLeft{3, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax backRight{4, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax indexer{5, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterRight{7, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterLeft{8, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder shooterRight_encoder = shooterRight.GetEncoder();

    frc::Joystick controls{5};
    Robot(){
        setData("Archibald", "Firestorm Robotics", 6341);
    }

    void Init(){

    }

    void TeleopLoop(){
        /*double limit = controls.GetThrottle();
        double turn = controls.GetX() * limit;
        double forw = controls.GetY() * limit;
        frontRight.Set(forw);
        frontLeft.Set(forw);
        backRight.Set(forw);
        backLeft.Set(forw);

        /*if (controls.GetTrigger()){
            indexer.Set(0.2);
        }
        shooterRight.Set(0.05);
        shooterLeft.Set(-0.05);
        usleep(100000);
        printf("%d\n", shooterRight_encoder.GetVelocity());*/
        double forThrust = controls.GetY() * controls.GetThrottle();
        double sideThrust = controls.GetX() * controls.GetThrottle();
        frontLeft.Set(forThrust + sideThrust);
        backLeft.Set(forThrust + sideThrust);
        frontRight.Set(-forThrust + sideThrust);
        frontRight.Set(-forThrust + sideThrust);
        if (controls.GetTrigger()){
            shooterRight.Set(1);
            shooterLeft.Set(-1);
        }
        else{
            shooterRight.Set(0);
            shooterLeft.Set(0);
        }
    }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
