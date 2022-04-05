#include <util/Motor.hpp>


struct DriveSide{
    Motor* front;
    Motor* back;
    double speed = 0;
    bool inverted = false;

    bool sticky = false; // If sticky is true, it keeps running at the set speed. Keep it at false for safety.

    void SetInverted(bool truth){
        inverted = truth;
    }

    void Drive(double value){
        if (inverted){
            speed -= value;
        }
        else{
            speed += value;
        }
    }

    void Apply(double limit = 1){
        front -> Set(speed * limit);
        back -> Set(speed * limit);
        if (!sticky){
            speed = 0;
        }
    }

    void Zero(){
        front -> Set(0);
        back -> Set(0);
        speed = 0;
    }
};


struct SimpleDrive {
    DriveSide left;
    DriveSide right;

    double speedLimit = 1;

    SimpleDrive(Motor* frontLeft, Motor* frontRight, Motor* backLeft, Motor* backRight){
        left.front = frontLeft;
        left.back = backLeft;
        right.front = frontRight;
        right.back = backRight;
        left.SetInverted (true);
    }

    void Forwards(double value){
        left.Drive(value);
        right.Drive(value);
    }

    void Turn(double value){
        left.Drive(value);
        right.Drive(-value);
    }

    void Backwards(double value){
        left.Drive(-value);
        right.Drive(-value);
    }

    void Apply(){
        left.Apply(speedLimit);
        right.Apply(speedLimit);
    }

    void TankLeft(double value){
        left.Drive(value);
    }

    void TankRight(double value){
        right.Drive(value);
    }

    void SetLimit(double value){
        speedLimit = value;
    }

    void Zero(){
        left.Zero();
        right.Zero();
    }
};
