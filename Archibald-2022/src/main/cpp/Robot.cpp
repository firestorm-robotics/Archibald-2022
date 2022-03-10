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

#define MODE_SHOOT           1
#define MODE_INTAKE_BALL     2
#define MODE_UNLOAD          3
#define MODE_MANUAL          4
#define MODE_DROP_INTAKE     5
#define MODE_RAISE_INTAKE    6
#define MODE_LOAD_TO_SHOOTER 7

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

    double customPIDPercentage = 0;

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

    //void customPIDSpeedTo(long speed);

    void drivePercent(double perc) {
        m_motor -> Set(perc);
    }

    long driveCustomPIDSpeed(long speed, double rampRate = 0.01){
        long shooterSpeed = m_encoder.GetVelocity();
        long speedDifference = speed - shooterSpeed;
        customPIDPercentage = (speedDifference / speed);// * rampRate; // Determine difference from optimal, like real PID. Change the 0.<x> thing as a ramp rate.
        m_motor -> Set(customPIDPercentage);
        return speedDifference;
    }

    void driveBy(long long pos) {
        done = false;
        reqPos = pos;
        m_encoder.SetPosition(0);
        m_pid.SetReference(pos, rev::ControlType::kPosition);
    }

    void drivePIDSpeed(long speed){
        m_pid.SetReference(speed, rev::ControlType::kVelocity);
    }

    int tick = 0;

    void runPID(){
        /*if (!done){
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
        }*/
    }

    long getSpeed() {
        return m_encoder.GetVelocity();
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


class Robot : public ModularRobot{
public:
    rev::CANSparkMax frontLeft    {MOTOR_FRONT_LEFT,    rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax frontRight   {MOTOR_FRONT_RIGHT,   rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax backLeft     {MOTOR_BACK_LEFT,     rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax backRight    {MOTOR_BACK_RIGHT,    rev::CANSparkMax::MotorType::kBrushless};

    EncoderifiedSpark* intakeEncoderifiedTest;
    EncoderifiedSpark* indexerEncoderified;

    SparkMaxDrive *drive;

    rev::CANSparkMax indexer      {MOTOR_INDEXER,       rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterRight {MOTOR_SHOOTER_RIGHT, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterLeft  {MOTOR_SHOOTER_LEFT,  rev::CANSparkMax::MotorType::kBrushless};

    //Superstructure* superstructure;


    rev::CANSparkMax intake {MOTOR_INTAKE, rev::CANSparkMax::MotorType::kBrushless};
    TalonSRX intakeDrop {MOTOR_INTAKE_DROP};

    rev::SparkMaxPIDController    shooterRightPID        = shooterRight.GetPIDController();
    rev::SparkMaxPIDController    shooterLeftPID         = shooterLeft.GetPIDController();

    rev::SparkMaxRelativeEncoder shooterRightEncoder = shooterRight.GetEncoder();

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


    frc::DigitalInput shooterPhotoelectric{7}; // When the higher laser (at the shooter) is interrupted by a ball, this reads high
    frc::DigitalInput intakePhotoelectric{8};  // When the lower laser (at the intake) is interrupted by a ball, this reads high

    frc::DigitalInput intakeUp{3};   // When the intake arm hits the upper limit, this reads high
    frc::DigitalInput intakeDown{4}; // When the intake arm hits the lower limit, this reads high

    frc::Joystick joystickControls{5};
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

    frc::GenericHID               controls{3};

    Robot() {
        setData("Archibald", "Firestorm Robotics", 6341);
        //shooterModeButton.set(&controls, 3);
        //indexerButton.set(&controls, 2);
        setPeriodicDelay(10000);
        intakeEncoderifiedTest = EncoderifiedSpark::createEncoderifiedSpark(&intake);
        indexerEncoderified = EncoderifiedSpark::createEncoderifiedSpark(&indexer);

        // Creates UsbCamera and MjpegServer [1] and connects them
        frc::CameraServer::StartAutomaticCapture();

        // Creates the CvSink and connects it to the UsbCamera
        cs::CvSink cvSink = frc::CameraServer::GetVideo();

        // Creates the CvSource and MjpegServer [2] and connects them
        cs::CvSource outputStream = frc::CameraServer::PutVideo("Blur", 800, 800);

        drive = new SparkMaxDrive(&frontLeft, &frontRight, &backLeft, &backRight);
        //shooterRight.SetInverted(false);
        //shooterLeft.SetInverted(false);
        //superstructure = new Superstructure(&intake, &intakeDrop, &indexer, &shooterRight, &shooterLeft, &intakePhotoelectric, &intakeDown, &intakeUp, &shooterPhotoelectric);
    }

    void loadShooter(long speed){
        shooterRightPID.SetReference(speed, rev::ControlType::kVelocity);
        shooterLeftPID.SetReference(-speed, rev::ControlType::kVelocity);
    }

    uint8_t shootMode = 0;

    bool loadBallToShooter(){
        if (shooterPhotoelectric.Get()){
            indexerEncoderified -> drivePercent(0);
            return true;
        }
        else{
            indexerEncoderified -> drivePercent(0.3);
        }
        return false;
    }

    uint8_t shootState = 0;

    bool shoot(long speed){
        if (shootState == 0){
            if (shooterPhotoelectric.Get()){
                shootState = 1;
                loadShooter(speed);
            }
            else {
                loadBallToShooter();
            }
        }
        else if (shootState == 1){
            if (shooterRightEncoder.GetVelocity() <= speed + 50){
                shootState = 2;
                printf("Phase 2\n");
            }
            else{
                intakeEncoderifiedTest -> drivePercent(0);
            }
        }
        else if (shootState == 2){
            if (!shooterPhotoelectric.Get()){
                shooterRight.Set(0);
                shooterLeft.Set(0);
                shootState = 0;
                intakeEncoderifiedTest -> drivePercent(0);
                return true;
            }
            else{
                intakeEncoderifiedTest -> drivePercent(0.5);
            }
        }
        return false;
    }

    bool dropIntake(){
        if (intakeDown.Get()){
            intakeDrop.Set(ControlMode::PercentOutput, 0);
            return true;
        }
        else{
            intakeDrop.Set(ControlMode::PercentOutput, -0.4);
        }
        return false;
    }

    bool raiseIntake(){
        if (intakeUp.Get()){
            intakeDrop.Set(ControlMode::PercentOutput, 0);
            return true;
        }
        else{
            intakeDrop.Set(ControlMode::PercentOutput, 1);
        }
        return false;
    }

    uint8_t intakeBallState = 0;

    bool intakeBall(){
        if (intakeBallState == 0){
            if (!intakePhotoelectric.Get()){
                intakeEncoderifiedTest -> drivePercent(-0.5);
            }
            else{
                intakeBallState = 1;
            }
        }
        else if (intakeBallState == 1){
            if (intakePhotoelectric.Get()){
                indexerEncoderified -> drivePercent(0.5);
            }
            else{
                indexerEncoderified -> drivePercent(0);
                intakeEncoderifiedTest -> drivePercent(0);
                intakeBallState = 0;
                return true;
            }
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
    bool xboxShot = false;

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
            intake.Set(-0.4);
            //intakeEncoderifiedTest -> driveBy(2);
        }
        else{
            intake.Set(0);
        }
        if (xboxControls.GetAButton()){
            //shooterRightPID.SetReference(NEO_500_RPM * 0.8, rev::ControlType::kVelocity);
            //shooterLeftPID.SetReference(NEO_500_RPM * 0.8, rev::ControlType::kVelocity);
            loadShooter(NEO_500_RPM * 0.8);
            xboxShot = true;
            printf("Xbox Shot\n");
        }
        else if (xboxShot){
            // I CAUGHT YOU HAHA
            shooterRight.Set(0);
            shooterLeft.Set(0);
            done2 = false;
            xboxShot = false;
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

    bool percentButtonboardShot = false;

    void buttonBoardMode(){
        double tankLeft = xboxControls.GetRawAxis(1);
        double tankRight = xboxControls.GetRawAxis(5);
        double limit = 0;//(controls.GetThrottle() + 1) / 2;
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
            //intakeEncoderifiedTest -> drivePercent(-0.4);
        }
        else{
            //intakeEncoderifiedTest -> drivePercent(0);
        }
        if (controls.GetRawButton(12) && thingitishi){
            thingitishi = false;
            drive -> leftPID(1);
            drive -> rightPID(-1);
            std::cout << "Who Cares" << std::endl;
        }
        if (joystickControls.GetRawButton(4)){
            shooterRight.Set(0.95);
            shooterLeft.Set(0.95);
            percentButtonboardShot = true;
        }
        else if (percentButtonboardShot){
            shooterRight.Set(0);
            shooterLeft.Set(0);
            percentButtonboardShot = false;
        }
        if (controls.GetRawButton(5)){
            //indexer.Set(controls.GetRawAxis(0) * -0.7);
            double intakeDropVal = -controls.GetRawAxis(0);
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
        //intakeEncoderifiedTest ->  runPID();
    }

    bool shootButtonState = false;
    bool shooting = false;

    bool test = true;

    bool joystickPIDShot = false;

    double variableSpeed = 0;

    void doubleMode(){
        //printf("Reading joystick values\n");
        double tankLeft = xboxControls.GetRawAxis(1);
        double tankRight = xboxControls.GetRawAxis(5);
        double limit = (joystickControls.GetThrottle() + 1) / 2;
        //printf("Running drivetrain\n");
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
        if (joystickControls.GetRawButton(12) || controls.GetRawButton(BUTTONBOARD_DROP_INTAKE)){
            thingitishi = false;
            mode = MODE_DROP_INTAKE;
        }
        //printf("Getting button 11\n");
        if (joystickControls.GetRawButton(11) || controls.GetRawButton(BUTTONBOARD_RAISE_INTAKE)){
            mode = MODE_RAISE_INTAKE;
        }
        //printf("Getting button 10\n");
        if (joystickControls.GetRawButton(10) || controls.GetRawButton(BUTTONBOARD_MACRO_INTAKE)){
            mode = MODE_INTAKE_BALL;
        }
        if (controls.GetRawButton(BUTTONBOARD_INDEXER_UP)){
            indexer.Set(0.5);
        }
        if (controls.GetRawButton(BUTTONBOARD_INDEXER_DOWN)){
            indexer.Set(-0.5);
        }
        if (controls.GetRawButton(BUTTONBOARD_SHOOTER)){
            shootButtonState = true;
        }
        else if (shootButtonState){
            shooting = !shooting;
            if (shooting) {
                if (controls.GetRawButton(BUTTONBOARD_SHOOTERSPEED_LOW)){
                    loadShooter(SHOOTERSPEED_LOW);
                }
                else if (controls.GetRawButton(BUTTONBOARD_SHOOTERSPEED_VARIABLE)){
                    loadShooter(NEO_500_RPM * controls.GetRawAxis(BUTTONBOARD_SHOOTER_AXIS));
                    variableSpeed = controls.GetRawAxis(BUTTONBOARD_SHOOTER_AXIS);
                }
                else{
                    loadShooter(SHOOTERSPEED_HIGH);
                }
            }
            else {
                loadShooter(0);
                shooterRight.Set(0);
                shooterLeft.Set(0);
            }
            shootButtonState = false;
        }
        if (shooting && controls.GetRawButton(BUTTONBOARD_SHOOTERSPEED_VARIABLE)){
            double tst = controls.GetRawAxis(BUTTONBOARD_SHOOTER_AXIS);
            if (tst > variableSpeed + 0.05 || tst < variableSpeed - 0.05){
                variableSpeed = tst;
                loadShooter(NEO_500_RPM * tst);
            }
        }
        //printf("Getting button 9\n");
        if (joystickControls.GetRawButton(9)){
            mode = MODE_LOAD_TO_SHOOTER;
        }
        //printf("Getting button 8\n");
        if (xboxControls.GetXButton()){
            if (test){
                shooterRightPID.SetReference(4000, rev::ControlType::kVelocity);
                shooterLeftPID.SetReference(-4000, rev::ControlType::kVelocity);
                printf("By all means, the test succeeded\n");
                test = false;
            }
            joystickPIDShot = true;
        }
        else if (joystickPIDShot){
            //shooterRight.Set(0);
            //shooterLeft.Set(0);
            joystickPIDShot = false;
            test = true; // Allow it to load PID again
        }
        //printf("Getting button 5\n");
        if (joystickControls.GetRawButton(5) || controls.GetRawButton(9)){
            //indexer.Set(joystickControls.GetRawAxis(0) * -0.7);
            double intakeDropVal = -joystickControls.GetY() + controls.GetRawAxis(2);
            if ((intakeDropVal < 0 || !intakeUp.Get()) && (intakeDropVal > 0 || !intakeDown.Get())) { // It can't keep moving up (positive) if it hits the limit, as demonstrated by this beeee-utiful or condition.
                intakeDrop.Set(ControlMode::PercentOutput, intakeDropVal * 0.8);
            }
        }
        else{
            //indexer.Set(0);
            intakeDrop.Set(ControlMode::PercentOutput, 0);
        }
        //printf("Getting y button\n");
        if (xboxControls.GetYButton()){
            mode = MODE_SHOOT;
        }
        //printf("Running indexer\n");
        indexer.Set((xboxControls.GetRawAxis(2) - xboxControls.GetRawAxis(3)) * 0.4);
        //printf("Running drivetrain\n");
        drive -> run();
        //intakeEncoderifiedTest ->  runPID();
        if (controls.GetRawButton(BUTTONBOARD_PANIC)){
            mode = MODE_MANUAL;
            printf("All macros ended\n");
        }
    }

    void BeginTeleop(){
        loadShooter(0);
    }

    void TeleopLoop(){
        /*if (controls.GetRawButton(6)){
            shootButtonState = true;
        }
        else if (shootButtonState){
            shootButtonState = false;
            mode = mode | MODE_SHOOT;
        }*/
        //printf("Double Mode\n");
        if (mode == MODE_MANUAL){
            doubleMode();
        }
        if (mode == MODE_SHOOT){
            if (shoot(NEO_500_RPM * 0.5)){
                mode = MODE_MANUAL;
                printf("Shot\n");
            }
        }
        if (mode == MODE_DROP_INTAKE){
            if (dropIntake()){
                mode = MODE_MANUAL;
                printf("Dropped intake\n");
            }
        }
        if (mode == MODE_RAISE_INTAKE){
            if (raiseIntake()){
                mode = MODE_MANUAL;
                printf("Intake done been rised\n");
            }
        }
        if (mode == MODE_INTAKE_BALL){
            doubleMode();
            if (intakeBall()){
                mode = MODE_MANUAL;
                printf("Intook ball\n");
            }
        }
        if (mode == MODE_LOAD_TO_SHOOTER){
            if (loadBallToShooter()){
                mode = MODE_MANUAL;
                printf("Loaded ball to shooter\n");
            }
        }
        if (mode == MODE_SHOOT){
            if (shoot(-2000)){
                mode = MODE_MANUAL;
                printf("Shot ball\n");
            }
        }
        //superstructure -> run();
    }
    void AutonomousLoop(){

    }
    void BeginTest(){
        //intakeEncoderifiedTest -> driveBy(1000);
    }
    void TestLoop(){
        for (int x = 1; x < 14; x ++){
            if (controls.GetRawButton(x)){
                std::cout << "Button " << x << " was just pushed" << std::endl;
                usleep(1000000);
            }
        }
    }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
