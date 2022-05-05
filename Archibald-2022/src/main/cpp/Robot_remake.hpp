#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>
#include "ModularRobot.hpp"
#include "rev/CANSparkMax.h"
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/DigitalInput.h>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>

#include <util/Motor.hpp>
#include <util/RunManager.hpp>
#include <util/functions.hpp>
#include "cleverName/Superstructure.hpp"
#include "cleverName/Drive.hpp"
#include "cleverName/Autonomous.hpp"
#include <util/ControlBoard.hpp>

#include <constants.h>


class Robot : public ModularRobot { // Create motors here, so we can have more control. And yes, lots of heap alloc; this aught to make Andrew happy.
    Motor* frontRight = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_FRONT_LEFT, rev::CANSparkMax::MotorType::kBrushless)); // You'll notice these are wrong; this is because the CAN ids are wrong and I don't want to kill v1.
    Motor* frontLeft = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_FRONT_RIGHT, rev::CANSparkMax::MotorType::kBrushless));
    Motor* backRight = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_BACK_LEFT, rev::CANSparkMax::MotorType::kBrushless));
    Motor* backLeft = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_BACK_RIGHT, rev::CANSparkMax::MotorType::kBrushless));
    Motor* indexerMotor = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_INDEXER, rev::CANSparkMax::MotorType::kBrushless));
    Motor* intakeMotor = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_INTAKE, rev::CANSparkMax::MotorType::kBrushless));
    Motor* intakeActuatorMotor = Motor::CreateMotor(new TalonSRX(MOTOR_INTAKE_DROP));
    Motor* shooterLeftMotor = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_SHOOTER_LEFT, rev::CANSparkMax::MotorType::kBrushless));
    Motor* shooterRightMotor = Motor::CreateMotor(new rev::CANSparkMax(MOTOR_SHOOTER_RIGHT, rev::CANSparkMax::MotorType::kBrushless));

    SimpleDrive drive;
    Superstructure superstructure;


    RunManager runner;

    ControlBoard controls;

    frc::DigitalInput photoElectric_intake {PHOTOELECTRIC_INTAKE};
    frc::DigitalInput photoElectric_shooter {PHOTOELECTRIC_SHOOTER};

    frc::DigitalInput intakeUpSwitch {LIMIT_INTAKE_UP};
    frc::DigitalInput intakeDownSwitch {LIMIT_INTAKE_DOWN};

    Autonomous leftTwoballAuto;

public:
    Robot() : drive (frontLeft, frontRight, backLeft, backRight), superstructure (indexerMotor, intakeActuatorMotor, intakeMotor, shooterLeftMotor, shooterRightMotor, &photoElectric_shooter, &photoElectric_intake, &intakeDownSwitch, &intakeUpSwitch){
        setData("Archibald", "Firestorm Robotics", 6341);
        runner.AddRunObject(&drive);
        runner.AddRunObject(&controls);
        runner.AddRunObject(&superstructure);

        leftTwoballAuto.AddStep(drive.CreateAutonomousToPosition(10));
        leftTwoballAuto.AddStep(drive.CreateAutonomousMoveToAngle(180, true));
    }

    void Init(){

    }

    void BeginTeleop(){
        drive.ZeroEncoders();
    }

    void TeleopLoop(){
        drive.TankLeft(controls.GetLeftY());
        drive.TankRight(controls.GetRightY());
        drive.SetLimit(controls.GetSpeedLimit());
        if (controls.GetIntakeMacroButtonReleased()){
            std::cout << "DEvin is sus" << std::endl;
            superstructure.DoIntakeIndex();
        }
        if (controls.GetShootMacroButtonReleased()){
            superstructure.Shoot(controls.GetShooterSpeed());
        }
        if (controls.GetIntakeUpButtonReleased()){
            superstructure.IntakeUp();
        }/*
        if (controls.GetIntakeDownButtonReleased()){
            superstructure.IntakeDown();
        }*/
    }

    void Loop(){
        runner.RunAll();
    }

    void AutonomousLoop(){

    }

    void BeginTest(){

    }

    void TestLoop(){
        leftTwoballAuto.Run();
    }
};


#pragma message ("You're using the remade robot.")
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
