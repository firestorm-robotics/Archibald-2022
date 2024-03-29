// Highly modular FRC robot class with threading support
// Basically a better version of Timed Robot.

#include <frc/RobotBase.h>
#include <vector>
#include <thread>
#include <atomic>
#include <string>
#include <frc/Notifier.h>
#include <units/time.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>

class ModularRobot;

class Module{ // Not much of a base class, but it serves a purpose!
protected:
    ModularRobot* myRobot;
public:
    void beginModule(ModularRobot* robot){
        myRobot = robot;
        init(robot);
    }
    virtual void init(ModularRobot* robot){

    }
    virtual void run(unsigned long long tick){

    }
};

class ModularRobot : public frc::RobotBase{
private:
    std::atomic<bool> m_exit{false};
    std::vector<Module> modules;
    std::string RobotName;
    std::string TeamName;
    int TeamNumber;
    unsigned long long tick; // Number of iterations since robot began
    unsigned long long localTick; // Number of iterations since current operation-mode began
    unsigned long long periodicDelayValue = 200000;
    const char* message;
public:
    uint8_t mode = 0; // 0 = disabled, 1 = autonomous, 2 = test, 3 = teleop
    void setData(const char* robotname, const char* teamname, int teamnumber){
        RobotName = robotname;
        TeamNumber = teamnumber;
        TeamName = teamname;
    }

    virtual void Init(){

    }

    virtual void BeginDisabled(){

    }

    virtual void DisabledLoop(){

    }

    virtual void CleanUpDisabled(){

    }

    virtual void BeginTeleop(){

    }

    virtual void TeleopLoop(){

    }

    virtual void CleanUpTeleop(){

    }

    virtual void BeginTest(){

    }

    virtual void TestLoop(){

    }

    virtual void CleanUpTest(){

    }

    virtual void BeginAutonomous(){

    }

    virtual void AutonomousLoop(){

    }

    virtual void CleanUpAutonomous(){

    }

    virtual void Loop(){

    } // User mainloop

    virtual void ItsOver(){

    }

    virtual void TeleopPeriodic(){

    }

    virtual void AutonomousPeriodic(){

    }

    virtual void TestPeriodic(){

    }

    void setPeriodicDelay(long micros){
        periodicDelayValue = micros;
    }

    static void periodicThread(ModularRobot *self){
        while (1){
            switch(self -> mode){
                case 1:
                    self -> AutonomousPeriodic();
                    break;
                case 2:
                    self -> TestPeriodic();
                    break;
                case 3:
                    self -> TeleopPeriodic();
                    break;
            }
            usleep(self -> periodicDelayValue);
        }
    }

    void periodicBegin(){
        std::thread periodic(periodicThread, this);
        periodic.detach();
    }

    void addModule(Module module){
        modules.push_back(module);
        module.beginModule(this); // Module init function should take a pointer to a ModularRobot
    }

    void loop(){
        for (Module i : modules){
            i.run(tick);
        }
        Loop();
    }

    void StartCompetition(){
        Init();
        std::cout << RobotName << " by " << TeamName << " (FRC " << TeamNumber << ") is now turning on!" << std::endl;
        HAL_InitializeDriverStation();
        HAL_ObserveUserProgramStarting();
        std::thread periodic(periodicThread, this);
        periodic.detach();
        while (!m_exit){ // Restructured from the old uglies. This one gives easy-peasy mainlooping without our ugly-mugly turdy-purdy stinky-winky infinite while loop
            loop(); // General mainloop
            if (IsDisabled()){ // Disabled tasks
                HAL_ObserveUserProgramDisabled();
                if (mode != 0){
                    HAL_SendConsoleLine("Begin Disable mode");
                    localTick = 0; // Reset the local tick counter
                    BeginDisabled();
                }
                if (mode == 1){
                    CleanUpAutonomous();
                }
                else if (mode == 2){
                    CleanUpTest();
                }
                else if (mode == 3){
                    CleanUpTeleop();
                }
                DisabledLoop();
                mode = 0;
            }
            else if (IsAutonomous()){ // Autonomous tasks
                HAL_ObserveUserProgramAutonomous();
                if (mode != 1){
                    HAL_SendConsoleLine("Begin Autonomous mode");
                    localTick = 0; // Reset the local tick counter
                    BeginAutonomous();
                }
                if (mode == 0){
                    CleanUpDisabled();
                }
                else if (mode == 2){
                    CleanUpTest();
                }
                else if (mode == 3){
                    CleanUpTeleop();
                }
                AutonomousLoop();
                mode = 1;
            }
            else if (IsTest()){ // Test tasks
                HAL_ObserveUserProgramTest();
                if (mode != 2){
                    HAL_SendConsoleLine("Begin test mode");
                    localTick = 0; // Reset the local tick counter
                    BeginTest();
                }
                if (mode == 0){
                    CleanUpDisabled();
                }
                else if (mode == 1){
                    CleanUpAutonomous();
                }
                else if (mode == 3){
                    CleanUpTeleop();
                }
                TestLoop();
                mode = 2;
            }
            else{ // Teleop tasks
                HAL_ObserveUserProgramTeleop();
                if (mode != 3){
                    HAL_SendConsoleLine("Begin Teleop mode");
                    localTick = 0; // Reset the local tick counter
                    BeginTeleop();
                }
                if (mode == 0){
                    CleanUpDisabled();
                }
                else if (mode == 1){
                    CleanUpAutonomous();
                }
                else if (mode == 2){
                    CleanUpTest();
                }
                TeleopLoop();
                mode = 3;
            }
            tick++; // Update the tick counters
            localTick++;
        }
        ItsOver();
    }
    void EndCompetition() {
        m_exit = true;
    }
};
