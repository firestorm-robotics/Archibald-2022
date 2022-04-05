#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>
#include "ModularRobot.hpp"
#include "rev/CANSparkMax.h"
#include <ctre/Phoenix.h>
#include <constants.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/DigitalInput.h>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>

#include <util/Motor.hpp>
#include <util/RunManager.hpp>
#include "cleverName/Intake.hpp"
#include "cleverName/Indexer.hpp"
#include "cleverName/Drive.hpp"


double sigmoid(double base, long long x){
    return (1/(1 + (1 / pow(base, x))) - 0.5) * 2; // Gets a sigmoid designed specifically for custom PID implementations
}

void setPIDPresets(rev::SparkMaxPIDController sparky){
    sparky.SetP (PID_kP);
    sparky.SetI (PID_kI);
    sparky.SetD (PID_kD);
    sparky.SetIZone (PID_kIz);
    sparky.SetFF (PID_kFF);
    sparky.SetOutputRange (PID_kMinOutput, PID_kMaxOutput);
}


class Robot : public ModularRobot {
    Motor* frontLeft = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_FRONT_LEFT, rev::CANSparkMax::MotorType::kBrushless));
    Motor* frontRight = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_FRONT_RIGHT, rev::CANSparkMax::MotorType::kBrushless));
    Motor* backLeft = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_BACK_LEFT, rev::CANSparkMax::MotorType::kBrushless));
    Motor* backRight = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_BACK_RIGHT, rev::CANSparkMax::MotorType::kBrushless));
    Motor* indexerMotor = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_INDEXER, rev::CANSparkMax::MotorType::kBrushless));
    Motor* intakeMotor = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_INTAKE, rev::CANSparkMax::MotorType::kBrushless));

    SimpleDrive drive;
    Intake intake;
    Indexer indexer;

    frc::Joystick joystick{1};
    frc::GenericHID buttonBoard{2};
    frc::XboxController xboxController{3};

    frc::DigitalInput photoElectric_intake {PHOTOELECTRIC_INTAKE};
    frc::DigitalInput photoElectric_shooter {PHOTOELECTRIC_SHOOTER};
public:
    Robot() : drive (frontLeft, frontRight, backLeft, backRight), intake (intakeMotor, &photoElectric_intake), indexer (indexerMotor, &photoElectric_intake, &photoElectric_shooter){
        setData("Archibald", "Firestorm Robotics", 6341);
    }

    void Init(){

    }

    void BeginTeleop(){
        intake.IntakeBall();
    }

    void TeleopLoop(){
        drive.TankLeft(xboxController.GetRawAxis(1));
        drive.TankRight(xboxController.GetRawAxis(5));
        drive.Apply();
        intake.Run();
    }

    void AutonomousLoop(){

    }

    void BeginTest(){

    }

    void TestLoop(){

    }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
