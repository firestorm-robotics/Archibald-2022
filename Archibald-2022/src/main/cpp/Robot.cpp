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

#define UNIVERSAL_DEADBAND 0.1 // The universal deadband for controls is 5%, this helps with our high-sensitivity issue.

#define NEO_500_RPM        5676

#define MODE_SHOOT         1
#define MODE_INTAKE        2
#define MODE_UNLOAD        4
#define MODE_MANUAL        8

#define SPARK_MAX_INTAKE



struct RobotButton {
    frc::Joystick *mJoy;
    int but;
    bool state = false;
    bool pressed = false;
    bool clicked = false;
    void set(frc::Joystick *joy, int button){
        but = button;
        mJoy = joy;
    }
    void run(){
        if (mJoy -> GetRawButton(but)){
            state = true;
            pressed = true;
        }
        else if (state){
            state = false;
            pressed = false;
            clicked = true;
        }
    }
    bool isPressed(){
        return pressed;
    }
    bool wasClicked(){
        if (clicked){
            clicked = false;
            return true;
        }
        else{
            return false;
        }
    }
};
/*
struct ToggleButton : RobotButton {
    bool on = false;
    void run(){
        if (mJoy -> GetRawButton(but)){
            state = true;
            pressed = true;
        }
        else if (state){
            state = false;
            pressed = false;
            clicked = true;
            on = !on;
        }
    }
};*/

class Robot : public ModularRobot{
public:
    // Try to keep it in rows in this class:
/*  ROW 1                         ROW 2                  ROW 3                 ROW 4                                     ROW 5*/
    rev::CANSparkMax frontLeft    {MOTOR_FRONT_LEFT,    rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax frontRight   {MOTOR_FRONT_RIGHT,   rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax backLeft     {MOTOR_BACK_LEFT,     rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax backRight    {MOTOR_BACK_RIGHT,    rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax indexer      {MOTOR_INDEXER,       rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterRight {MOTOR_SHOOTER_RIGHT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterLeft  {MOTOR_SHOOTER_LEFT,  rev::CANSparkMax::MotorType::kBrushless};
    #ifdef SPARK_MAX_INTAKE
    rev::CANSparkMax intake       {MOTOR_INTAKE,        rev::CANSparkMax::MotorType::kBrushless};
    #endif
    #ifdef TALON_SRX_INTAKE
    TalonSRX intake {MOTOR_INTAKE};
    #endif
    TalonSRX intakeDrop {MOTOR_INTAKE_DROP};

    rev::SparkMaxPIDController    shooterRightPID        =                     shooterRight.GetPIDController();
    rev::SparkMaxPIDController    shooterLeftPID         =                     shooterLeft.GetPIDController();
    rev::SparkMaxPIDController    indexerPID             =                     indexer.GetPIDController();

    rev::SparkMaxRelativeEncoder  shooterLeftEncoder     =                     shooterLeft.GetEncoder();
    rev::SparkMaxRelativeEncoder  indexerEncoder         =                     indexer.GetEncoder();
    const double                  kP                     =                     6e-5,
                                  kI                     =                     1e-6,
                                  kD                     =                     0,
                                  kIz                    =                     0,
                                  kFF                    =                     0.000015,
                                  kMaxOutput             =                     1.0,
                                  kMinOutput             =                     -1.0;


    frc::DigitalInput switcheroo{2};

    frc::Joystick controls{5};
    frc::XboxController xboxControls{4};
    RobotButton shooterModeButton;//{controls, 3};
    uint32_t                      mode                   =                     MODE_MANUAL;

    bool                          runningShooter         =                     false;
    bool                          triggerClick           =                     false;

    RobotButton indexerButton;//(controls, 2);

    uint8_t                       shooterMode            =                     0;        // 0 = max speed, 1 = variable mode
    uint8_t                       maxMode                =                     1;
    bool                          shooterModeButtonState =                     false;
    int                           times                  =                     0;

    Robot(){
        setData(                  "Archibald",           "Firestorm Robotics", 6341);
        shooterModeButton.set(&controls, 3);
        indexerButton.set(&controls, 2);
    }

    void loadShooter(long speed){
        shooterRightPID.SetReference(speed, rev::ControlType::kVelocity);
        shooterLeftPID.SetReference(speed, rev::ControlType::kVelocity);
    }

    bool shoot(long speed){
        static uint8_t shootMode = 0;
        if (shootMode == 0){
            loadShooter(speed);
            indexerPID.SetReference(2000, rev::ControlType::kVelocity);
            shootMode = 1;
        }
        else if (shootMode == 1){
            if (shooterLeftEncoder.GetVelocity() >= speed - 100){
                printf("Phase 2\n");
                shootMode = 2;
            }
        }
        else if (shootMode == 2){
            if (switcheroo.Get()){
                shootMode = 3;
            }
        }
        else if (shootMode == 3){
            if (!switcheroo.Get()){
                shootMode = 4;
            }
        }
        else if (shootMode == 4){
            indexer .Set(0);
            shooterRight.Set(0);
            shooterLeft.Set(0);
            shootMode = 0;
            return true;
        }
        return false;
    }

    void Init(){
        shooterRightPID .SetP                  (kP);
        shooterRightPID .SetI                  (kI);
        shooterRightPID .SetD                  (kD);
        shooterRightPID .SetIZone              (kIz);
        shooterRightPID .SetFF                 (kFF);
        shooterRightPID .SetOutputRange        (kMinOutput,          kMaxOutput);
        shooterLeftPID .SetP                  (kP);
        shooterLeftPID .SetI                  (kI);
        shooterLeftPID .SetD                  (kD);
        shooterLeftPID .SetIZone              (kIz);
        shooterLeftPID .SetFF                 (kFF);
        shooterLeftPID .SetOutputRange        (kMinOutput,          kMaxOutput);
        indexerPID     .SetP                  (kP);
        indexerPID     .SetI                  (kI);
        indexerPID     .SetD                  (kD);
        indexerPID     .SetIZone              (kIz);
        indexerPID     .SetFF                 (kFF);
        indexerPID     .SetOutputRange        (kMinOutput,          kMaxOutput);
    }

    bool runningIndexer = false;

    void TeleopLoop(){
        /*if (controls.GetRawButton(6)){
            shootButtonState = true;
        }
        else if (shootButtonState){
            shootButtonState = false;
            mode = mode | MODE_SHOOT;
        }*/
        if (mode & MODE_MANUAL){
            double limit = (controls.GetThrottle() + 1) / 2;
            double forThrust = controls.GetY() * limit;
            double sideThrust = controls.GetX() * limit;
            if (runningIndexer && (fabs(controls.GetTwist() > UNIVERSAL_DEADBAND))){
                indexer.Set(controls.GetTwist() * limit);
                frontLeft.Set(0);
                frontRight.Set(0);
                backLeft.Set(0);
                backRight.Set(0);
            }
            else if ((fabs(controls.GetX()) + fabs(controls.GetY())) > UNIVERSAL_DEADBAND){
                indexer.Set(0);
                frontLeft.Set(forThrust + sideThrust);
                backLeft.Set(forThrust + sideThrust);
                frontRight.Set(-forThrust + sideThrust);
                frontRight.Set(-forThrust + sideThrust);
            }
            else{
                indexer.Set(0);
                frontLeft.Set(0);
                backLeft.Set(0);
                frontRight.Set(0);
                frontRight.Set(0);
            }
            indexerButton.run();
            if (indexerButton.wasClicked()){
                runningIndexer = !runningIndexer;
            }
            if (controls.GetTriggerReleased()){
                runningShooter = !runningShooter;
            }
            if (runningShooter){
                static bool shooterSet = false;
                if (shooterMode == 0){
                    shooterRightPID.SetReference(NEO_500_RPM * 0.9, rev::ControlType::kVelocity);
                    shooterLeftPID.SetReference(NEO_500_RPM * 0.9, rev::ControlType::kVelocity);
                }
                else if (shooterMode == 1){
                    shooterRightPID.SetReference(NEO_500_RPM * 0.8, rev::ControlType::kVelocity);
                    shooterLeftPID.SetReference(NEO_500_RPM * 0.8, rev::ControlType::kVelocity);
                    shooterMode = 0;
                }
            }
            else{
                shooterRight.Set(0);
                shooterLeft.Set(0);
            }
            shooterModeButton.run();
            if (shooterModeButton.wasClicked()){
                shooterMode ++;
                if (shooterMode > maxMode){
                    shooterMode = 0;
                }
            }
            if (controls.GetRawButton(4)){
                #ifdef TALON_SRX_INTAKE
                intake.Set(ControlMode::PercentOutput, -0.50);
                #endif
                #ifdef SPARK_MAX_INTAKE
                intake.Set(-0.5);
                #endif
            }
            else{
                #ifdef TALON_SRX_INTAKE
                intake.Set(ControlMode::PercentOutput, 0);
                #endif
                #ifdef SPARK_MAX_INTAKE
                intake.Set(0);
                #endif
            }
        }
        if (mode & MODE_SHOOT){
            if (shoot(NEO_500_RPM * 0.8)){
                mode &= ~MODE_SHOOT;
                printf("Not Shootin' No More\n");
            }
        }
    }
    void AutonomousLoop(){
        //intakeDrop.Set(ControlMode::PercentOutput, -0.25);
        indexer.Set(-0.3);
    }
    void TestLoop(){
        frontLeft.Set(xboxControls.GetRawAxis(5) * 0.25);
        backLeft.Set(xboxControls.GetRawAxis(5) * 0.25);
        frontRight.Set(xboxControls.GetRawAxis(1) * -0.25);
        backRight.Set(xboxControls.GetRawAxis(1) * -0.25);
        intakeDrop.Set(ControlMode::PercentOutput, xboxControls.GetRawAxis(0) * 0.6);
        //if (xboxControls.GetRawButton(2)){
            indexer.Set(0.3);
            usleep(1000000);
            printf("It's working, I hope\n");
    }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
