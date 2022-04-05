#include <util/Motor.hpp>
#include <frc/DigitalInput.h>


#define INDEXER_MODE_NONE     0
#define INDEXER_MODE_INTAKING 1
#define INDEXER_MODE_SHOOTING 2

struct Indexer {
    Motor* indexerMotor;
    frc::DigitalInput* ps_lowerPhotoelectric;
    frc::DigitalInput* ps_higherPhotoelectric;

    uint8_t balls = 0;
    uint8_t opMode = INDEXER_MODE_NONE;

    bool ballEntering = false;
    bool ballShooting = false;

    Indexer(Motor* motor, frc::DigitalInput* lowerPhotoelectric, frc::DigitalInput* higherPhotoelectric) {
        ps_lowerPhotoelectric = lowerPhotoelectric;
        ps_higherPhotoelectric = higherPhotoelectric;
        indexerMotor = motor;
    }

    void Run(){
        if (opMode == INDEXER_MODE_SHOOTING){
            indexerMotor -> Set(0.3);
        }
        else if (opMode == INDEXER_MODE_INTAKING){
            indexerMotor -> Set(0.5);
        }
        if (ps_lowerPhotoelectric -> Get() && !ballEntering){
            ballEntering = true;
        }
        else if (!ps_lowerPhotoelectric -> Get() && ballEntering){
            ballEntering = false;
            _ballIn();
        }
        if (ps_higherPhotoelectric -> Get() && !ballShooting){
            ballShooting = true;
        }
        else if (!ps_higherPhotoelectric -> Get() && ballShooting){
            ballShooting = false;
            _ballShot();
        }
    }

    void _ballIn(){
        balls ++;
        if (opMode == INDEXER_MODE_INTAKING){
            Stop();
        }
        else {
            printf("An error didn't occur.\nNo, I'm not kidding. An error didn't occur.\nStop it, an error didn't occur.\nFine: Error code 13.11 occurred.\nERROR\n");
        }
    }

    void _ballShot(){
        balls --;
        if (opMode == INDEXER_MODE_SHOOTING){
            Stop();
        }
        else {
            printf("ERROR ERROR ERROR\nERROR ERROR ERROR\nERROR ERROR ERROR\n        ERROR CODE 10.5445 OCCURRED\n ERROR\n");
        }
    }

    void Stop(){
        indexerMotor -> Set(0);
        opMode = INDEXER_MODE_NONE;
    }

    void RaiseBallToShooter() {
        opMode = INDEXER_MODE_SHOOTING;
    }

    void GrabBallFromIntake() {
        opMode = INDEXER_MODE_INTAKING;
    }
};
