// Extension of Modular Robot with a focus on state-based actions and custom modes.


#include <map>
#include "ModularRobot.hpp"
#include <functional>

class ActionRobot : public ModularRobot{
private:
    std::map <unsigned int, std::function<void(void)>> modes;
    std::map <unsigned int, std::function<bool(void)>> actions;
    unsigned int curAction = 0;
    unsigned int curMode = 0;
public:
    void RegisterMode(unsigned int mode, std::function<void(void)> modeFun){
        modes[mode + 1] = modeFun;
    }
    void RegisterAction(unsigned int action, std::function<bool(void)> actionFun){
        actions[action + 1] = actionFun;
    }
    void Loop(){
        if (curAction > 0){
            if (actions[curAction]()){
                curAction = 0;
            }
        }
        else if (curMode > 0){
            modes[curMode]();
        }
    }
    bool TriggerAction(unsigned int num, bool block = false){
        if (curAction != 0){
            if (block){
                while (curAction != 0){
                    Loop();
                }
            }
            else{
                return false;
            }
        }
        else{
            curAction = num;
        }
    }
    bool EndAction(){
        curAction = 0;
    }
};
