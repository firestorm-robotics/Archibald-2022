/* By Tyler Clarke. Any external contributions, under any circumstances or for any reason whatsoever will be acknowledged here.
    Intake, Index, Shoot.
*/

#include <util/Motor.hpp>
#include <util/RunManager.hpp>


struct IntakeGroup {
    Motor* actuator;
    Motor* buzz;
    frc::DigitalInput* photoelectric;
    frc::DigitalInput* intakeUp;
    frc::DigitalInput* intakeDown;

    bool Raise(){
        actuator -> Set(0.3);
        if (intakeUp -> Get()){
            actuator -> Set(0);
            return true;
        }
        return false;
    }

    bool Lower(){
        actuator -> Set(-0.3);
        if (intakeDown -> Get()){
            actuator -> Set(0);
            return true;
        }
        return false;
    }

    bool Grab(){
        buzz -> Set(0.3);
        if (photoelectric -> Get()){
            buzz -> Set(0);
            return true;
        }
        return false;
    }
};


struct ShooterGroup {
    Motor* left;
    Motor* right;
    frc::DigitalInput* photoelectric;

    double shootSpeed = 0; // Won't shoot without speed on.

    bool ballIn = false;

    uint8_t DoShoot(long speed){
        bool leftAtSpeed = left -> SetSpeed(speed);
        bool rightAtSpeed = right -> SetSpeed(speed);
        if (leftAtSpeed && rightAtSpeed){ // They be upramped.
            if (photoelectric -> Get() && !ballIn){
                ballIn = true;
                return 0; // 0 = The ball is ready to shoot
            }
            else if (ballIn){
                ballIn = false;
                left -> Set(0); // Zero them.
                right -> Set(0);
                return 1; // 1 = Shot the ball
            }
        }
        return 2; // 2 = Didn't do anything
    }

    bool Shoot(){
        if (DoShoot(shootSpeed) == 1){
            return true;
        }
        return false;
    }

    void SetSpeed(double speed){
        shootSpeed = speed;
    }
};


struct Indexer{
    Motor* motor;
    frc::DigitalInput* higherPhotoelectric;
    frc::DigitalInput* lowerPhotoelectric;

    bool ballEntering = false;
    bool ballExiting = false;

    bool inverse = false; // If we decide to dump any balls, inverse becomes true.

    int ballsCount = 0;

    bool gotBall = false;

    void DoBallCheck(){
        if (higherPhotoelectric -> Get()){
            ballExiting = true;
        }
        else if (ballExiting){
            ballsCount --;
            ballExiting = false;
        }
        if (lowerPhotoelectric -> Get()){
            ballEntering = true;
        }
        else if (ballEntering){
            ballEntering = false;
            if (inverse){
                ballsCount --;
            }
            else{
                ballsCount ++;
                gotBall = true;
            }
        }
    }

    bool Grab(){
        inverse = false;
        motor -> Set(0.3);
        if (gotBall){
            gotBall = false;
            motor -> Set(0);
            return true;
        }
        return false;
    }
};


struct Superstructure : public Runnable{
    IntakeGroup intake;
    Indexer indexer;
    ShooterGroup shooter;

    int balls;

    bool waitingForBall = true;

    bool intaking = false;

    bool shooting = false;

    bool raisingIntake = false;

    Superstructure(Motor* indexerMotor,
        Motor* intakeActuatorMotor,
        Motor* intakeMotor,
        Motor* shooterLeft,
        Motor* shooterRight,
        frc::DigitalInput* higherPhotoelectric,
        frc::DigitalInput* lowerPhotoelectric,
        frc::DigitalInput* lowerLimitswitch,
        frc::DigitalInput* higherLimitswitch)
    {
        AssignIndexerMotor(indexerMotor);
        AssignIntakeMotor(intakeMotor);
        AssignIntakeActuatorMotor(intakeActuatorMotor);
        AssignShooterMotors(shooterLeft, shooterRight);
        AssignHigherPhotoelectric(higherPhotoelectric);
        AssignLowerPhotoelectric(lowerPhotoelectric);
        AssignLimitSwitches(lowerLimitswitch, higherLimitswitch);
    }

    Superstructure(){ /* Empty constructor for people who use Assign commands. */

    }

    void AssignIndexerMotor(Motor* indexerMotor){
        indexer.motor = indexerMotor;
    }

    void AssignIntakeActuatorMotor(Motor* intakeActuatorMotor){
        intake.actuator = intakeActuatorMotor;
    }

    void AssignIntakeMotor(Motor* intakeMotor){
        intake.buzz = intakeMotor;
    }

    void AssignShooterMotors(Motor* left, Motor* right){
        shooter.left = left;
        shooter.right = right;
    }

    void AssignHigherPhotoelectric(frc::DigitalInput* sensor){
        shooter.photoelectric = sensor;
        indexer.higherPhotoelectric = sensor;
    }

    void AssignLowerPhotoelectric(frc::DigitalInput* sensor){
        intake.photoelectric = sensor;
        indexer.lowerPhotoelectric = sensor;
    }

    void AssignLimitSwitches(frc::DigitalInput* lower, frc::DigitalInput* higher){
        intake.intakeUp = higher;
        intake.intakeDown = lower;
    }

    bool IntakeAndIndex(){
        if (waitingForBall){
            if (intake.Grab()){
                waitingForBall = false;
            }
        }
        else {
            if (indexer.Grab()){
                waitingForBall = true;
                return true;
            }
        }
        return false;
    }

    void DoIntakeIndex(){
        intaking = true;
    }

    void Shoot(double speed){
        shooting = true;
        shooter.SetSpeed(speed);
    }

    void IntakeUp(){
        raisingIntake = true;
    }

    void Run(){
        if (intaking){
            if (IntakeAndIndex()){
                std::cout << "Done" << std::endl;
                intaking = false;
            }
        }

        if (shooting){
            if (shooter.Shoot()){
                shooting = false;
            }
        }

        if (raisingIntake){
            if (intake.Raise()){
                raisingIntake = false;
            }
        }

        indexer.DoBallCheck();
        balls = indexer.ballsCount;
    }
};
