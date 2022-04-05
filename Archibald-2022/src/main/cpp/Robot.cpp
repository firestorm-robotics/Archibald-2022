#define OLDMODE

#ifdef NEWMODE
#include "Robot_remake.hpp"
#endif

#ifdef OLDMODE
#include "networktables/NetworkTable.h"
//#include "networktables/NetworkTableEntry.h"
//#include "networktables/NetworkTableInstance.h"

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
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "rev/REVLibError.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "AHRS.h"
#include <frc/SPI.h>


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

double matchColors(frc::Color color, frc::Color color2){
    double redDif = fabs(color.red - color2.red);
    double blueDif = fabs(color.blue - color2.blue);
    double greenDif = fabs(color.green - color2.green);
    double avg = (redDif + blueDif + greenDif) / 3;
    return avg;
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
        m_pid.SetReference(pos, rev::CANSparkMax::ControlType::kPosition);
    }

    void drivePIDSpeed(long speed){
        m_pid.SetReference(speed, rev::CANSparkMax::ControlType::kVelocity);
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

    void resetEncoders(){
        if (eS_frontLeft -> m_encoder.SetPosition(0) != rev::REVLibError::kOk){
            std::cout << "Got an error on the front left encoder" << std::endl;
        }
        if (eS_frontRight -> m_encoder.SetPosition(0) != rev::REVLibError::kOk){
            std::cout << "Got an error on the front right encoder" << std::endl;
        }
        if (eS_backLeft -> m_encoder.SetPosition(0) != rev::REVLibError::kOk){
            std::cout << "Got an error on the back left encoder" << std::endl;
        }
        if (eS_backRight -> m_encoder.SetPosition(0) != rev::REVLibError::kOk){
            std::cout << "Got an error on the back right encoder" << std::endl;
        }
    }

    void printEncoders(){
        std::cout << "Front Right Encoder: " << eS_frontRight -> m_encoder.GetPosition() << std::endl;
    }

    bool runToRots(long rots, double reduction = 0.1){
        double distR = eS_frontRight -> m_encoder.GetPosition();
        double wrongR = rots - distR;
        double wrongRPerc = sigmoid(10, wrongR) * reduction; //(wrong / rots) * reduction;

        double distL = -eS_frontLeft -> m_encoder.GetPosition();
        frc::SmartDashboard::PutNumber("Dist L", distL);
        double wrongL = rots - distL;
        double wrongLPerc = sigmoid(10, wrongL) * reduction; //(wrong / rots) * reduction;
        percentDifferentialRight(wrongRPerc);
        percentDifferentialLeft(-wrongLPerc);
        run();
        if (fabs(wrongR) < 5/* && fabs(wrongL) < 5*/){
            percentArcadeForwards(0);
            return true;
        }
        return false;
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

    //Timer time;

    //Superstructure* superstructure;


    rev::CANSparkMax intake {MOTOR_INTAKE, rev::CANSparkMax::MotorType::kBrushless};
    TalonSRX intakeDrop {MOTOR_INTAKE_DROP};
    VictorSPX leftStaticHook {MOTOR_STATICHOOK_LEFT};
    VictorSPX rightStaticHook {MOTOR_STATICHOOK_RIGHT};

    TalonFX climb {MOTOR_CLIMB};

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


    frc::DigitalInput shooterPhotoelectric{PHOTOELECTRIC_SHOOTER}; // When the higher laser (at the shooter) is interrupted by a ball, this reads high
    frc::DigitalInput intakePhotoelectric{PHOTOELECTRIC_INTAKE};  // When the lower laser (at the intake) is interrupted by a ball, this reads high

    frc::DigitalInput intakeUp{3};   // When the intake arm hits the upper limit, this reads high
    frc::DigitalInput intakeDown{4}; // When the intake arm hits the lower limit, this reads high
    frc::DigitalInput climberDown{CLIMBERSWITCH};

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
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
    rev::ColorSensorV3 m_colorSensor{i2cPort};

    frc::Color redBall;
    frc::Color blueBall;
    frc::Color blackTape;
    rev::ColorMatch m_colorMatcher;

    AHRS* navX;

    double startAng = 0;

    //frc::ShuffleboardTab shuffleAutoTab = frc::Shuffleboard::GetTab("Autonomous metrics");


    Robot() {
        setData("Archibald", "Firestorm Robotics", 6341);
        //shooterModeButton.set(&controls, 3);
        //indexerButton.set(&controls, 2);
        setPeriodicDelay(2000);
        intakeEncoderifiedTest = EncoderifiedSpark::createEncoderifiedSpark(&intake);
        indexerEncoderified = EncoderifiedSpark::createEncoderifiedSpark(&indexer);

        // Creates UsbCamera and MjpegServer [1] and connects them

        drive = new SparkMaxDrive(&frontLeft, &frontRight, &backLeft, &backRight);
        //shooterRight.SetInverted(false);
        //shooterLeft.SetInverted(false);
        //superstructure = new Superstructure(&intake, &intakeDrop, &indexer, &shooterRight, &shooterLeft, &intakePhotoelectric, &intakeDown, &intakeUp, &shooterPhotoelectric);
        setPIDPresets(shooterLeftPID);
        setPIDPresets(shooterRightPID);

        climb.SetNeutralMode(NeutralMode::Brake);
        m_colorMatcher.AddColorMatch(redBall);
        redBall.red = 0.53;
        redBall.green = 0.344;
        redBall.blue = 0.125;

        blueBall.red = 0.160;
        blueBall.green = 0.409;
        blueBall.blue = 0.430;

        blackTape.red = 0.255;
        blackTape.green = 0.48;
        blackTape.blue = 0.264;
        navX = new AHRS(frc::SPI::Port::kMXP);
    }

    void loadShooter(long speed){
        shooterRightPID.SetReference(speed, rev::CANSparkMax::ControlType::kVelocity);
        shooterLeftPID.SetReference(speed, rev::CANSparkMax::ControlType::kVelocity);
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
            loadShooter(speed);
            shootState = 1;
        }
        else if (shootState == 1){
            if (shooterRightEncoder.GetVelocity() >= speed - PID_ACCEPTABLE_ERROR){
                shootState = 2;
                printf("Phase 2\n");
            }
            else{
                indexerEncoderified -> drivePercent(0);
            }
        }
        else if (shootState == 2){
            if (shooterPhotoelectric.Get()){
                shootState = 3;
            }
            else {
                indexerEncoderified ->  drivePercent(0.3);
            }
        }
        else if (shootState == 3){
            if (!shooterPhotoelectric.Get()){
                shooterRight.Set(0);
                shooterLeft.Set(0);
                shootState = 0;
                indexerEncoderified -> drivePercent(0);
                return true;
            }
            else{
                indexerEncoderified -> drivePercent(0.5);
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
            intakeDrop.Set(ControlMode::PercentOutput, -0.7);
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
                intakeEncoderifiedTest -> drivePercent(0.5);
            }
            else{
                intakeBallState = 1;
            }
        }
        else if (intakeBallState == 1){
            if (intakePhotoelectric.Get()){
                intakeBallState = 2;
            }
            intakeEncoderifiedTest -> drivePercent(0.5);
            indexerEncoderified -> drivePercent(0.6);
        }
        else if (intakeBallState == 2){
            if (!intakePhotoelectric.Get()){
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
        double limit = 0.6;//(controls.GetThrottle() + 1) / 2;
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
            shooterRight.Set(0.97);
            shooterLeft.Set(0.97);
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
        //indexer.Set((xboxControls.GetRawAxis(2) - xboxControls.GetRawAxis(3)) * 0.4);
        drive -> run();
        //intakeEncoderifiedTest ->  runPID();
    }

    bool shootButtonState = false;
    bool shooting = false;

    bool test = true;

    bool joystickPIDShot = false;

    double variableSpeed = 0;

    bool manualIndexerRan = false;

    bool needsNominalNotification = false;

    bool manualIntakeRun = true;

    long shooterReqSpeed = 0;

    double climberUpSpeed = 0;

    bool manualEject = false;

    void doubleMode(){
        //printf("Reading joystick values\n");
        double tankLeft = xboxControls.GetRawAxis(1);
        double tankRight = xboxControls.GetRawAxis(5);
        if (xboxControls.GetLeftBumper() || xboxControls.GetRightBumper()){
            tankLeft = 0;
            tankRight = 0;
        }
        //double limit = (joystickControls.GetThrottle() + 1) / 2;
        double limit = (controls.GetRawAxis(0) + 1) / 2;
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
        if (/*joystickControls.GetRawButton(12) || */controls.GetRawButton(BUTTONBOARD_DROP_INTAKE)){
            thingitishi = false;
            mode = MODE_DROP_INTAKE;
        }
        //printf("Getting button 11\n");
        if (/*joystickControls.GetRawButton(11) || */controls.GetRawButton(BUTTONBOARD_RAISE_INTAKE)){
            mode = MODE_RAISE_INTAKE;
        }
        //printf("Getting button 10\n");
        if (/*joystickControls.GetRawButton(10) || */controls.GetRawButton(BUTTONBOARD_MACRO_INTAKE)){
            mode = MODE_INTAKE_BALL;
        }
        if (controls.GetRawButton(BUTTONBOARD_INDEXER_UP)/* || joystickControls.GetRawButton(JOYSTICK_INDEXER_UP)*/){
            indexer.Set(0.2);
            manualIndexerRan = true;
        }
        else if (controls.GetRawButton(BUTTONBOARD_INDEXER_DOWN) /*|| joystickControls.GetRawButton(JOYSTICK_INDEXER_DOWN)*/){
            indexer.Set(-0.2);
            manualIndexerRan = true;
        }
        else if (manualIndexerRan) {
            indexer.Set(0);
            manualIndexerRan = false;
        }
        if (controls.GetRawButton(BUTTONBOARD_SHOOTER)){
            loadShooter(shooterReqSpeed);
            needsNominalNotification = true;
            frc::SmartDashboard::PutBoolean("Shooting", true);
            shooting = true;
        }
        else if (shooting) {
            shooting = false;
            loadShooter(0);
            shooterRight.Set(0);
            shooterLeft.Set(0);
            frc::SmartDashboard::PutBoolean("Shooting", false);
        }
        /*if (controls.GetRawButton(BUTTONBOARD_SHOOTERSPEED_VARIABLE)){
            double tst = (controls.GetRawAxis(BUTTONBOARD_SHOOTER_AXIS) + 1) / 2;
            if (tst > variableSpeed + 0.05 || tst < variableSpeed - 0.05){
                variableSpeed = tst;
                loadShooter(NEO_500_RPM * tst);
            }
        }*/
        if (controls.GetRawButton(BUTTONBOARD_SHOOTERSPEED_VARIABLE)){
            shooterReqSpeed = SHOOTERSPEED_MID;//((controls.GetRawAxis(BUTTONBOARD_SHOOTER_AXIS) + 1) / 2) * NEO_500_RPM;
        }
        else if (controls.GetRawButton(BUTTONBOARD_SHOOTERSPEED_LOW)){
            shooterReqSpeed = SHOOTERSPEED_LOW;
        }
        else{
            shooterReqSpeed = SHOOTERSPEED_HIGH;
        }
        if (fabs(shooterReqSpeed - shooterRightEncoder.GetVelocity()) <= PID_ACCEPTABLE_ERROR){
            if (needsNominalNotification){
                printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
                printf(" YOU HAVE ACHIEVED NOMINAL SPEED \n");
                printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
                needsNominalNotification = false;
            }
            frc::SmartDashboard::PutBoolean("Shooter Ready", true);
        }
        else {
            needsNominalNotification = true; // When it goes out of the acceptable range, it resets the nominal notification.
            frc::SmartDashboard::PutBoolean("Shooter Ready", false);
        }
        //printf("Getting button 9\n");
        if (false/*joystickControls.GetRawButton(9)*/){
            mode = MODE_LOAD_TO_SHOOTER;
        }
        //printf("Getting button 8\n");
        /*if (xboxControls.GetXButton()){
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
        }*/
        //printf("Getting button 5\n");
        if (false/*joystickControls.GetRawButton(5)*/){
            //indexer.Set(joystickControls.GetRawAxis(0) * -0.7);
            double intakeDropVal = -joystickControls.GetY() + controls.GetRawAxis(2);
            if ((intakeDropVal < 0 || !intakeUp.Get()) && (intakeDropVal > 0 || !intakeDown.Get())) { // It can't keep moving up (positive) if it hits the limit, as demonstrated by this beeee-utiful or condition.
                intakeDrop.Set(ControlMode::PercentOutput, intakeDropVal * 0.8);
            }
        }
        /*else{
            //indexer.Set(0);
            intakeDrop.Set(ControlMode::PercentOutput, 0);
        }*/
        //printf("Getting y button\n");
        /*if (xboxControls.GetYButton()){
            mode = MODE_SHOOT;
        }*/
        //printf("Running indexer\n");
        //indexer.Set((xboxControls.GetRawAxis(2) - xboxControls.GetRawAxis(3)) * 0.4);
        //printf("Running drivetrain\n");
        if (xboxControls.GetXButton()){
            double limelightTargetOffsetAngle = table->GetNumber("tx",0.0);
            double dif = (limelightTargetOffsetAngle) / 27;
            frontLeft.Set((dif * 0.5) - 0.35);
            backLeft.Set((dif * 0.5) - 0.35);
            frontRight.Set((dif * 0.5) + 0.35);
            backRight.Set((dif * 0.5) + 0.35);
        }
        else if (xboxControls.GetAButton()){
            frontLeft.Set(0.2);
            frontRight.Set(-0.2);
            backLeft.Set(0.2);
            backRight.Set(-0.2);
        }
        else{
            drive -> run();
        }
        //intakeEncoderifiedTest ->  runPID();
        if (controls.GetRawButton(BUTTONBOARD_PANIC)){
            mode = MODE_MANUAL;
            intake.Set(0);
            indexer.Set(0);
            shooterLeft.Set(0);
            shooterRight.Set(0);
            printf("PANNIICCCCC\nPANNNNIICCCCCC\n\nWhich is to say: you panicked and deactivated all the motors on the robot.\n");
        }
        if (controls.GetRawButton(BUTTONBOARD_SHOOTER_MACRO)){
            mode = MODE_SHOOT;
        }
        if (controls.GetRawButton(7)){
            holdSpeed = xboxControls.GetRawAxis(2) - xboxControls.GetRawAxis(3);
            climb.Set(ControlMode::PercentOutput, holdSpeed);
            if (climberDown.Get()){
                if (climberUpSpeed == 0){
                    climberUpSpeed = holdSpeed;
                }
                climb.Set(ControlMode::PercentOutput, climberUpSpeed * -1);
            }
            else {
                climberUpSpeed = 0;
            }
        }
        else{
            climb.Set(ControlMode::PercentOutput, 0.01);
        }
        if (xboxControls.GetBButton()){
            manualEject = true;
            indexer.Set(-0.3);
            intake.Set(-0.3);
        }
        else if (manualEject){
            indexer.Set(0);
            intake.Set(0);
            manualEject = false;
        }
        if (xboxControls.GetRightBumper()){
            leftStaticHook.Set(ControlMode::PercentOutput, -xboxControls.GetRawAxis(5));
            rightStaticHook.Set(ControlMode::PercentOutput, xboxControls.GetRawAxis(1));
        }
        else {
            leftStaticHook.Set(ControlMode::PercentOutput, 0);
            rightStaticHook.Set(ControlMode::PercentOutput, 0);
        }
        frc::SmartDashboard::PutNumber("Direction", fixAngle(navx_GetAngle()));//navx_GetAngle());
        frc::SmartDashboard::PutBoolean("Intake Touching Up Switch", intakeUp.Get());
        frc::SmartDashboard::PutBoolean("Intake Touching Down Switch", intakeDown.Get());
    }

    double holdSpeed = 0;

    void BeginTeleop(){
        loadShooter(0);
        startAng = navx_GetAngle();
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
        doubleMode();
        if (mode == MODE_SHOOT){
            if (shoot(shooterReqSpeed)){
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
        /*if (mode == MODE_SHOOT){
            if (shoot(-2000)){
                mode = MODE_MANUAL;
                printf("Shot ball\n");
            }
        }*/
        //superstructure -> run();
    }

    uint8_t autonomousPhase = 0;
    // One ball auto
    /*void AutonomousLoop(){
        if (autonomousPhase == 1){
            frontLeft.Set(-0.3);
            backLeft.Set(-0.3);
            frontRight.Set(0.3);
            backRight.Set(0.3);
        }
        std::cout << rotsToInches(drive -> eS_backLeft -> m_encoder.GetPosition()) << std::endl;
        usleep(1000000);
    }*/

    double rotsToInches(double rots){
        return (rots * 18.5) / 6;
    }
    // One ball auto
    /*void BeginAutonomous(){
        while (!shoot(SHOOTERSPEED_HIGH));
        usleep(1000000); // Give it a very short amount of time.
        drive -> percentArcadeForwards(0.3);
        drive -> run();
        usleep(1000000);
        drive -> percentArcadeForwards(0);
        drive -> run();
        //drive -> eS_backLeft -> m_encoder.SetPosition(0);
    }*/

    const int shooterTestVelocity = 1000;

    /*void TestLoop() {
        //std::cout << time.Get() << std::endl;
        usleep(1000000);
    }*/

    uint8_t autoPhase_l = 0;

    bool driveToAngle(double ang, double speed = 0.2, bool direction = false){
        if (ang < 0){
            ang *= -1;
            //direction = !direction;
        }
        double difAng = navx_GetAngle() - ang;
        double power = 0;
        if (difAng > 20){
            power = speed * (direction ? 1 : -1);
        }
        else if (difAng < -20){
            power = -speed * (direction ? 1 : -1);
        }
        else if (difAng > 10){
            power = speed/2 * (direction ? 1 : -1);
        }
        else if (difAng < -10){
            power = -speed/2 * (direction ? 1 : -1);
        }
        else if (difAng > 5){
            power = speed/4 * (direction ? 1 : -1);
        }
        else if (difAng < -5){
            power = -speed/4 * (direction ? 1 : -1);
        }
        else {
            drive -> percentArcadeTurn(0);
            drive -> run();
            return true; // It no longer has to run the motors, it's such a big boy now!
        }
        drive -> percentArcadeTurn(power);
        drive -> run();
        return false;
    }

    double fixAngle(double angle){ // Coterminal!
        angle -= startAng;
        while (angle < 0){
            angle += 360;
        }
        while (angle > 360){
            angle -= 360;
        }
        return angle;
    }
    // Two ball auto - right
    void AutonomousLoop() {
        //double limelightTargetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
        //std::cout << limelightTargetOffsetAngle_Vertical << std::endl;
        //usleep(1000000);
        /*frc::Color detectedColor = m_colorSensor.GetColor();
        //std::cout << matchColors(detectedColor, blueBall) << std::endl;
        double isBlackTape = matchColors(detectedColor, blackTape);
        double isRedBall = matchColors(detectedColor, redBall);
        double isBlueBall = matchColors(detectedColor, blueBall);
        if (isBlackTape < 0.003){
            std::cout << "I see Black Tape" << std::endl;
        }
        else {
            std::cout << "I see nothing." << std::endl;
        }
        std::cout << isBlueBall << std::endl;
        //std::cout << "R: " << detectedColor.red << " G: " << detectedColor.green << " B: " << detectedColor.blue << std::endl;
        usleep(1000000);*/
        if (autoPhase_l == 0){
            if (shoot(SHOOTERSPEED_LOW)){
                autoPhase_l ++;
            }
        }
        else if (autoPhase_l == 1){
            if (drive -> runToRots(-34)){
                autoPhase_l ++;
                startAng = navx_GetAngle();
            }
        }
        else if (autoPhase_l == 2){ // Robot is at 78 degrees at start (33 + 45), needs to turn to the ball which is (from trig) 21, so turn it by 90 - 78 = 12 degrees then turn by 21. The total change is 33. We want it to be in the 4th quadrant, so 180 -.
            if (driveToAngle(fixAngle(180 - 55))) {
                autoPhase_l ++;
            }
        }
        else if (autoPhase_l == 3){
            if (dropIntake()){
                autoPhase_l ++;
                drive -> resetEncoders();
            }
        }
        else if (autoPhase_l == 4){
            drive -> percentArcadeForwards(-0.08);
            drive -> run();
            if (intakeBall()){
                autoPhase_l ++;
                drive -> percentArcadeForwards(0);
                drive -> run();
            }
        }
        else if (autoPhase_l == 5){
            if (raiseIntake()){
                autoPhase_l ++;
            }
        }
        else if (autoPhase_l == 6){
            if (drive -> runToRots(0)){
                autoPhase_l ++;
            }
        }
        else if (autoPhase_l == 7){
            if (driveToAngle(175)) {
                autoPhase_l ++;
                drive -> resetEncoders();
            }
        }
        else if (autoPhase_l == 8){
            if (drive -> runToRots(37)){
                autoPhase_l ++;
                drive -> percentArcadeForwards(0);
                drive -> run();
            }
        }
        else if (autoPhase_l == 9){
            if (shoot(SHOOTERSPEED_LOW)){
                autoPhase_l ++;
            }
        }
    }

    double navx_GetAngle(){
        return navX -> GetYaw() + 180;
    }
    // Two ball auto - right
    void BeginAutonomous(){
        drive -> resetEncoders();
        drive -> printEncoders();
        navX -> ZeroYaw();
        autoPhase_l = 0;
        startAng = navx_GetAngle();
    }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

#endif
