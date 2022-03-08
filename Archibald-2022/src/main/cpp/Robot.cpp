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
#include "cameraserver/CameraServer.h"

#define UNIVERSAL_DEADBAND 0.1 // The universal deadband for controls is 5%, this helps with our high-sensitivity issue.

#define NEO_500_RPM        5676

#define MODE_SHOOT         1
#define MODE_INTAKE        2
#define MODE_UNLOAD        4
#define MODE_MANUAL        8
#define MODE_FORWARDS      16

#define SPARK_MAX_INTAKE // The intake is handled by a spark not a talon. This does stuff to the constructor.


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

float timedifference_msec(struct timeval t0, struct timeval t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
}


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

struct EncoderifiedSpark{
    rev::CANSparkMax* m_motor;
    rev::SparkMaxPIDController m_pid;
    rev::SparkMaxRelativeEncoder m_encoder;
    bool done;
    long long reqPos;
    long long encoderPos;
    int error;

    /*EncoderifiedSpark(rev::CANSparkMax motor, rev::SparkMaxPIDController pid, rev::SparkMaxRelativeEncoder encoder) {
        //m_motor = motor;
        //m_pid = &pid;
        //m_encoder = &encoder;
        //setPIDPresets(m_pid);
    }*/

    /*EncoderifiedSpark(int canID) : m_motor(canID, rev::CANSparkMax::MotorType::kBrushless), m_pid = m_motor -> GetPIDController(){
        rev::SparkMaxPIDController tempPIDCont = m_motor -> GetPIDController();
        m_pid = &tempPIDCont;
        rev::SparkMaxRelativeEncoder tempEncoder = m_motor -> GetEncoder();
        m_encoder = &tempEncoder;
    }

    void SetSparkMax(rev::CANSparkMax* motor, rev::SparkMaxPIDController pid, rev::SparkMaxRelativeEncoder encoder) {
        m_motor = motor;
        m_pid = &pid;
        m_encoder = &encoder;
        setPIDPresets(m_pid);
    }*/

    static EncoderifiedSpark* createEncoderifiedSpark(rev::CANSparkMax* motor){
        EncoderifiedSpark* returner = (EncoderifiedSpark*)malloc(sizeof(EncoderifiedSpark));
        returner -> m_motor = motor;
        returner -> m_pid = motor -> GetPIDController();
        returner -> m_encoder = motor -> GetEncoder();
        returner -> done = true;
        returner -> reqPos = 0;
        returner -> error = 50;
        setPIDPresets(returner -> m_pid);
        return returner;
    }

    void setPIDError(int amount){
        error = amount;
    }

    void drivePercent(double perc) {
        m_motor -> Set(perc);
    }

    void driveBy(long long pos) {
        done = false;
        reqPos = pos;
        m_encoder.SetPosition(0);
        m_pid.SetReference(pos, rev::ControlType::kPosition);
    }

    int tick = 0;

    void runPID(){
        if (!done){
            tick ++;
            if (tick == 1000){
                tick = 0;
                encoderPos = m_encoder.GetPosition();
                m_pid.SetReference(reqPos, rev::ControlType::kPosition);
                if ((encoderPos > reqPos - error) && (encoderPos < reqPos + error)){
                    std::cout << "A motor finished a task!" << std::endl;
                    done = true;
                }
            }
        }
    }
};

struct SparkMaxDrive{
    rev::CANSparkMax* m_frontLeft;
    rev::CANSparkMax* m_frontRight;
    rev::CANSparkMax* m_backLeft;
    rev::CANSparkMax* m_backRight;
    double frontLeftPercent = 0;
    double frontRightPercent = 0;
    double backLeftPercent = 0;
    double backRightPercent = 0;
    EncoderifiedSpark* eS_frontLeft;
    EncoderifiedSpark* eS_frontRight;
    EncoderifiedSpark* eS_backLeft;
    EncoderifiedSpark* eS_backRight;

    bool pid = false; // It uses percent mode for now.

    SparkMaxDrive(rev::CANSparkMax* frontLeft, rev::CANSparkMax* frontRight, rev::CANSparkMax* backLeft, rev::CANSparkMax* backRight) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_backLeft = backLeft;
        m_backRight = backRight;
        eS_frontLeft = EncoderifiedSpark::createEncoderifiedSpark(m_frontLeft);
        eS_frontRight = EncoderifiedSpark::createEncoderifiedSpark(m_frontRight);
        eS_backLeft = EncoderifiedSpark::createEncoderifiedSpark(m_backLeft);
        eS_backRight = EncoderifiedSpark::createEncoderifiedSpark(m_backRight);
    }

    void beginPIDSampling(){
        pid = true;
    }

    void beginPercent(){
        pid = false;
    }

    void percentDifferentialLeft(double val){
        frontLeftPercent += val;
        backLeftPercent += val;
    }

    void percentDifferentialRight(double val){
        frontRightPercent += val;
        backRightPercent += val;
    }

    void percentArcadeForwards(double val){
        frontLeftPercent += val;
        backLeftPercent += val;
        frontRightPercent -= val;
        backRightPercent -= val;
    }

    void percentArcadeTurn(double val){
        frontLeftPercent += val;
        backLeftPercent += val;
        frontRightPercent += val;
        backRightPercent += val;
    }

    void leftPID(long long amount){
        //eS_frontLeft -> driveBy(amount);
        eS_backLeft -> driveBy(amount);
    }

    void rightPID(long long amount){
        //eS_frontRight -> driveBy(amount);
        //eS_backRight -> driveBy(amount);
    }

    void run(){
        if (pid){
            eS_frontLeft -> runPID();
            eS_frontRight -> runPID();
            eS_backLeft -> runPID();
            eS_backRight -> runPID();
        }
        else{
            eS_frontLeft -> drivePercent(frontLeftPercent);
            frontLeftPercent = 0;
            eS_frontRight -> drivePercent(frontRightPercent);
            frontRightPercent = 0;
            eS_backLeft -> drivePercent(backLeftPercent);
            backLeftPercent = 0;
            eS_backRight -> drivePercent(backRightPercent);
            backRightPercent = 0;
        }
    }
};


struct Shooter{
    Shooter(rev::CANSparkMax shooterRight, rev::CANSparkMax shooterLeft){

    }
};


class Robot : public ModularRobot{
public:
    rev::CANSparkMax frontLeft    {MOTOR_FRONT_LEFT,    rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax frontRight   {MOTOR_FRONT_RIGHT,   rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax backLeft     {MOTOR_BACK_LEFT,     rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax backRight    {MOTOR_BACK_RIGHT,    rev::CANSparkMax::MotorType::kBrushless};

    EncoderifiedSpark* intakeEncoderifiedTest;

    SparkMaxDrive *drive;

    rev::CANSparkMax indexer      {MOTOR_INDEXER,       rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterRight {MOTOR_SHOOTER_RIGHT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterLeft  {MOTOR_SHOOTER_LEFT,  rev::CANSparkMax::MotorType::kBrushless};


    #ifdef SPARK_MAX_INTAKE
    rev::CANSparkMax intake {MOTOR_INTAKE, rev::CANSparkMax::MotorType::kBrushless};
    #endif
    #ifdef TALON_SRX_INTAKE
    TalonSRX intake {MOTOR_INTAKE};
    #endif
    TalonSRX intakeDrop {MOTOR_INTAKE_DROP};

    rev::SparkMaxPIDController    shooterRightPID        = shooterRight.GetPIDController();
    rev::SparkMaxPIDController    shooterLeftPID         = shooterLeft.GetPIDController();
    rev::SparkMaxPIDController    indexerPID             = indexer.GetPIDController();

    /*rev::SparkMaxRelativeEncoder  shooterLeftEncoder     = shooterLeft.GetEncoder();
    rev::SparkMaxRelativeEncoder  shooterRightEncoder    = shooterRight.GetEncoder();
    rev::SparkMaxRelativeEncoder  indexerEncoder         = indexer.GetEncoder();*/
    /*const double                  kP                     = 6e-5,
                                  kI                     = 1e-6,
                                  kD                     = 0,
                                  kIz                    = 0,
                                  kFF                    = 0.000015,
                                  kMaxOutput             = 0.2,
                                  kMinOutput             = -0.2;*/


    frc::DigitalInput switcheroo{2};
    frc::DigitalInput intakeUp{3};
    frc::DigitalInput intakeDown{4};

    frc::Joystick controls{5};
    frc::XboxController xboxControls{4};
    RobotButton shooterModeButton;
    uint32_t                      mode                   =                     MODE_MANUAL;

    bool                          runningShooter         =                     false;
    bool                          triggerClick           =                     false;

    RobotButton indexerButton;

    uint8_t                       shooterMode            =                     0;        // 0 = max speed, 1 = variable mode
    uint8_t                       maxMode                =                     1;
    bool                          shooterModeButtonState =                     false;
    int                           times                  =                     0;

    frc::GenericHID               buttonBoard{3};

    Robot() {
        setData("Archibald", "Firestorm Robotics", 6341);
        shooterModeButton.set(&controls, 3);
        indexerButton.set(&controls, 2);
        setPeriodicDelay(10000);
        intakeEncoderifiedTest = EncoderifiedSpark::createEncoderifiedSpark(&intake);

        // Creates UsbCamera and MjpegServer [1] and connects them
        frc::CameraServer::StartAutomaticCapture();

        // Creates the CvSink and connects it to the UsbCamera
        cs::CvSink cvSink = frc::CameraServer::GetVideo();

        // Creates the CvSource and MjpegServer [2] and connects them
        cs::CvSource outputStream = frc::CameraServer::PutVideo("Blur", 800, 800);

        drive = new SparkMaxDrive(&frontLeft, &frontRight, &backLeft, &backRight);
    }

    void loadShooter(long speed){
        shooterRightPID.SetReference(speed, rev::ControlType::kVelocity);
        shooterLeftPID.SetReference(-speed, rev::ControlType::kVelocity);
    }

    uint8_t shootMode = 0;

    bool shoot(long speed){
        if (shootMode == 0){ // Leave empty for now
            loadShooter(speed);
            shootMode = 1;
        }
        else if (shootMode == 1){
            if (true/*shooterRightEncoder.GetVelocity() >= speed - 200*/){
                printf("Phase 2\n");
                indexerPID.SetReference(2000, rev::ControlType::kVelocity);
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
                indexer.Set(0); // Leave PID mode.
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

    uint8_t distMode = 0;
    long reqPosFrontRight = 0;

    void Init(){

    }

    bool runningIndexer = false;
    bool done = false;
    bool done2 = false;

    void xboxMode(){
        double xAxis = xboxControls.GetRawAxis(0);
        double yAxis = xboxControls.GetRawAxis(1);
        double tankLeft = yAxis + xAxis;
        double tankRight = yAxis - xAxis;
        frontLeft.Set(tankLeft * 0.25);
        backLeft.Set(tankLeft * 0.25);
        frontRight.Set(tankRight * -0.25);
        backRight.Set(tankRight * -0.25);
        double intakeDropVal = -xboxControls.GetRawAxis(5);
        if ((intakeDropVal < 0 || !intakeUp.Get()) && (intakeDropVal > 0 || !intakeDown.Get())) { // It can't keep moving up (positive) if it hits the limit, as demonstrated by this beeee-utiful or condition.
            intakeDrop.Set(ControlMode::PercentOutput, intakeDropVal * 0.6);
        }
        if (xboxControls.GetAButton()){
            //intake.Set(-0.4);
            intakeEncoderifiedTest -> driveBy(2);
        }
        else{
            //intake.Set(0);
        }
        if (xboxControls.GetRawButton(2)){
            /*shooterRightPID.SetReference(NEO_500_RPM * 0.8, rev::ControlType::kVelocity);
            shooterLeftPID.SetReference(NEO_500_RPM * 0.8, rev::ControlType::kVelocity);*/
            loadShooter(NEO_500_RPM * 0.8);
        }
        else{
            shooterRight.Set(0);
            shooterLeft.Set(0);
            done2 = false;
        }
        if (xboxControls.GetYButton()){
            mode |= MODE_SHOOT; // Shoot mode
            mode &= ~MODE_MANUAL; // Manual and Shoot can't run at the same time
        }
        /*if (xboxControls.GetXButton() && !done){
            done = true;
            mode = MODE_FORWARDS;
            //mode &= ~MODE_MANUAL;
        }*/
        indexer.Set((xboxControls.GetRawAxis(2) - xboxControls.GetRawAxis(3)) * 0.5);
    }
    bool thingitishi = true;

    void doubleMode(){
        double tankLeft = xboxControls.GetRawAxis(1);
        double tankRight = xboxControls.GetRawAxis(5);
        double limit = (controls.GetThrottle() + 1) / 2;
        if (fabs(tankLeft) > UNIVERSAL_DEADBAND){
            drive -> percentDifferentialRight(-tankLeft * limit);
        }
        else{
            drive -> percentDifferentialRight(0);
        }
        if (fabs(tankRight) > UNIVERSAL_DEADBAND){
            drive -> percentDifferentialLeft(tankRight * limit);
        }
        else {
            drive -> percentDifferentialLeft(0);
        }
        if (controls.GetRawButton(3)){
            intakeEncoderifiedTest -> drivePercent(-0.4);
        }
        else{
            intakeEncoderifiedTest -> drivePercent(0);
        }
        if (controls.GetRawButton(12) && thingitishi){
            thingitishi = false;
            drive -> leftPID(1);
            drive -> rightPID(-1);
            std::cout << "Who Cares" << std::endl;
        }
        if (controls.GetTrigger()){
            shooterRight.Set(0.95);
            shooterLeft.Set(0.95);
        }
        else{
            shooterRight.Set(0);
            shooterLeft.Set(0);
        }
        if (controls.GetRawButton(6)){
            //indexer.Set(controls.GetY() * -0.7);
            double intakeDropVal = -controls.GetY();
            if ((intakeDropVal < 0 || !intakeUp.Get()) && (intakeDropVal > 0 || !intakeDown.Get())) { // It can't keep moving up (positive) if it hits the limit, as demonstrated by this beeee-utiful or condition.
                intakeDrop.Set(ControlMode::PercentOutput, intakeDropVal * 0.8);
            }
        }
        else{
            //indexer.Set(0);
            intakeDrop.Set(ControlMode::PercentOutput, 0);
        }
        if (xboxControls.GetYButton()){
            mode = MODE_SHOOT;
        }
        indexer.Set((xboxControls.GetRawAxis(2) - xboxControls.GetRawAxis(3)) * 0.4);
        drive -> run();
        intakeEncoderifiedTest ->  runPID();
    }

    void TeleopLoop(){
        /*if (controls.GetRawButton(6)){
            shootButtonState = true;
        }
        else if (shootButtonState){
            shootButtonState = false;
            mode = mode | MODE_SHOOT;
        }*/
        if (mode & MODE_MANUAL){
            doubleMode();
        }
        if (mode & MODE_SHOOT){
            if (shoot(NEO_500_RPM * 0.5)){
                mode &= ~MODE_SHOOT;
                mode |= MODE_MANUAL;
                printf("Shot\n");
            }
        }
        /*if (mode & MODE_FORWARDS){
            if (moveDistance(200)){
                //mode &= ~MODE_FORWARDS;
                mode = MODE_MANUAL;
            }
        }*/
    }
    void AutonomousLoop(){

    }
    void BeginTest(){
        intakeEncoderifiedTest -> driveBy(1000);
    }
    void TestLoop(){
        if (buttonBoard.GetRawButton(1)){
            std::cout << "It works!" << std::endl;
        }
    }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
