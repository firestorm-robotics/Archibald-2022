#ifndef DRIVE_HPP
#define DRIVE_HPP
#include <util/Motor.hpp>
#include <util/functions.hpp>
#include <util/RunManager.hpp>
#include "AHRS.h"
#include "Autonomous.hpp"

/*
struct SimpleDrive : public Runnable{
    void ZeroEncoders();
    bool ToRots(double);
}; // Forward dec
*/

struct DriveSide{
    Motor* front;
    Motor* back;
    double speed = 0;
    bool inverted = false;

    bool sticky = false; // If sticky is true, it keeps running at the set speed. Keep it at false for safety.

    DriveSide(Motor* frontMotor, Motor* backMotor){
        front = frontMotor;
        back = backMotor; // We may eventually use locks, I don't intend to for now; if the jitter issue becomes real, I will.
    }

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

    bool ToRots(double rots){
        front -> SetPos(rots);
        back -> SetPos(rots);
        if (fabs(front -> GetPosition() - rots) < PID_POSITION_ACCEPTABLE_ERROR){
            return true;
        }
        return false;
    }

    void ZeroEncoders(){
        front -> ZeroEncoder();
        back -> ZeroEncoder();
    }

    double GetPos(){
        return front -> GetPosition();// + back -> GetPosition()) / 2;
    }

    void Stop(){
        front -> Set(0);
        back -> Set(0);
        speed = 0;
    }

    bool IsZeroed(){
        return front -> IsZeroed() && back -> IsZeroed();
    }

    double GetSpeed(){
        return front -> GetVelocity();
    }
};


struct SimpleDrive : public Runnable{
    DriveSide left;
    DriveSide right;
    AHRS* navX;

    uint8_t driveMode = 0;

    double speedLimit = 1;

    SimpleDrive(Motor* frontLeft, Motor* frontRight, Motor* backLeft, Motor* backRight) : left(frontLeft, backLeft), right(frontRight, backRight){
        left.SetInverted (true);
        navX = new AHRS(frc::SPI::Port::kMXP);
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

    void Run(){
        if (driveMode == 0){
            left.Apply(speedLimit);
            right.Apply(speedLimit);
        }
    }

    void TankLeft(double value){
        driveMode = 0;
        left.Drive(value);
    }

    void TankRight(double value){
        driveMode = 0;
        right.Drive(value);
    }

    void SetLimit(double value){
        speedLimit = value;
    }

    void Zero(){
        left.Zero();
        right.Zero();
    }

    bool ToRotsLeft(double rots){
        return left.ToRots(rots);
    }

    bool ToRotsRight(double rots){
        return right.ToRots(rots);
    }

    bool ToRots(double rots){
        driveMode = 1;
        bool toRotsLeft = ToRotsLeft(rots);
        bool toRotsRight = ToRotsRight(-rots); // Right side is wrong
        return toRotsLeft && toRotsRight; // Have to do separate statements to avoid optimization! Normally if ToRotsLeft returns wrong, the other side of the && isn't checked so ToRotsRight never gets a chance.
    }

    void ZeroEncoders(){
        right.ZeroEncoders();
        left.ZeroEncoders();
        std::cout << "Zero encoders" << std::endl;
    }

    double GetLeftPosition(){
        return left.GetPos();
    }

    double GetRightPosition(){
        return right.GetPos();
    }

    double GetAveragePosition(){
        return (GetLeftPosition() - GetRightPosition()) / 2; // Invert for Right; because Right is Wrong.
    }

    double GetLeftSpeed(){
        return left.GetSpeed();
    }

    double GetRightSpeed(){
        return right.GetSpeed();
    }

    double GetAverageSpeed(){
        return (fabs(GetLeftSpeed()) + fabs(GetRightSpeed())) / 2;
    }

    double GetRotation(){
        return navX -> GetYaw();
    }

    bool ToAngle(double angle, double error = 0.01, bool magic = true, double magicTolerance = 1){
        double angleDist = util::minAngleDistance(GetRotation(), angle);
        double fractionDist = angleDist/360;
        double computedDist = fractionDist; // This is redundancy for extra math later if I want it
        if (fabs(computedDist) > error){
            if (magic){
                if (GetAverageSpeed() < magicTolerance){
                    computedDist *= 2;
                }
            }
            Turn(computedDist);
        }
        else{
            return true;
        }
        return false;
    }

    void Stop(){
        left.Stop();
        right.Stop();
    }

    bool IsZeroed(){
        return left.IsZeroed() && right.IsZeroed();
    }

    AutonomousStep* CreateAutonomousToPosition(double);

    AutonomousStep* CreateAutonomousMoveToAngle(double, bool);
};


struct AutonomousToPositionEntity : public AutonomousRunnable{
    long pos;
    SimpleDrive* m_drive;
    bool ready = false;
    void Begin(){
        m_drive -> ZeroEncoders();
        std::cout << "Beginning Autonomous to Position Entity" << std::endl;
        std::cout << "Your encoder position is: " << m_drive -> GetAveragePosition() << std::endl;
    }
    bool Run(){
        if (ready){
            return m_drive -> ToRots(pos);
        }
        else{
            if (m_drive -> IsZeroed()){ // Basically wait until the encoders are zero, because they don't update instantly.
                ready = true;
            }
        }
        return false;
    }
    void Cleanup(){
        m_drive -> Stop();
        std::cout << "I'm cleaning up, I want the world to know I've got to let it show" << std::endl;
    }
    std::string Identify(){
        return "Autonomous To Position Entity";
    }
};


struct AutonomousToAngleEntity : public AutonomousRunnable{
    SimpleDrive* m_drive;
    double ang;
    bool rel;
    double startAng = 0;
    void Begin(){
        if (rel){
            startAng = m_drive -> GetRotation();
        }
    }
    bool Run(){
        if (m_drive -> ToAngle(ang - startAng)){
            return true;
        }
        return false;
    }
    std::string Identify(){
        return "Autonomous To Angle Entity";
    }
};


AutonomousStep* SimpleDrive::CreateAutonomousToPosition(double position){
    AutonomousToPositionEntity* entity = new AutonomousToPositionEntity();
    entity -> pos = position;
    entity -> m_drive = this;
    return new AutonomousStep(entity);
}

AutonomousStep* SimpleDrive::CreateAutonomousMoveToAngle(double angle, bool relative = true){
    AutonomousToAngleEntity* entity = new AutonomousToAngleEntity();
    entity -> ang = angle;
    entity -> rel = relative;
    entity -> m_drive = this;
    return new AutonomousStep(entity);
}
#endif
