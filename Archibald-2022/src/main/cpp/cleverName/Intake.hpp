#include <util/Motor.hpp>
#include <frc/DigitalInput.h>

#define INTAKE_MODE_NONE           0
#define INTAKE_MODE_INTAKING       1
#define INTAKE_MODE_RUN_TO_INDEXER 2

#include <util/RunManager.hpp>

struct Intake : Runnable {
    Motor* intakeMotor;
    frc::DigitalInput* photoElectric;

    uint8_t mode = INTAKE_MODE_NONE;

    bool hasBall = true;

    Intake(Motor* motor, frc::DigitalInput* ballInSensor){
        intakeMotor = motor;
        photoElectric = ballInSensor;
    }

    bool CheckHasBall(){
        return photoElectric -> Get();
    }

    void IntakeBall(){
        mode = INTAKE_MODE_INTAKING;
    }

    void Run(){
        if (mode == INTAKE_MODE_INTAKING){
            intakeMotor -> Set(0.4);
            if (CheckHasBall()){
                Stop();
            }
        }
        else if (mode == INTAKE_MODE_RUN_TO_INDEXER){
            intakeMotor -> Set(0.4);
            if (!CheckHasBall()){
                Stop();
            }
        }
    }

    void Stop(){
        intakeMotor -> Set(0);
        mode = INTAKE_MODE_NONE;
    }

    void RunIntoIndexer(){
        mode = INTAKE_MODE_RUN_TO_INDEXER;
    }
};
