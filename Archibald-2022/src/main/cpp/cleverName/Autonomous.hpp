/* By Tyler Clarke, any external contributions under any circumstances or for any reasons whatsoever will be acknowledged here.
    I'm really excited about this one - modular Autonomous that doesn't suck!
*/
#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP
#include <vector>


struct AutonomousRunnable { // Base class.
    AutonomousRunnable(){

    }

    bool run(){ // Maybe add some other stuff here later, otherwise I wouldn't mess with dumb encapsulation steps :)
        return Run();
    }

    virtual void Begin(){

    }

    virtual void Cleanup(){

    }

    virtual bool Run(){
        return true;
    }

    virtual std::string Identify(){
        return "Generic Autonomous Runnable";
    }
};


struct AutonomousStep {
    AutonomousRunnable* runner;
    unsigned long long maxFrameCount = 0;

    AutonomousStep(AutonomousRunnable* run, unsigned long long maxFrames = 0){
        maxFrameCount = maxFrames;
        runner = run;
    }
};


struct Autonomous {
    std::vector<AutonomousStep*> steps;
    unsigned int stepNum = 0;
    unsigned long long totalFrames = 0;
    bool done = false;
    bool initialized = false;

    Autonomous(){

    }

    void AddStep(AutonomousStep* step){
        steps.push_back(step);
        std::cout << "Added a " << step -> runner -> Identify() << " to the autonomous." << std::endl;
    }

    void Reset(){
        done = false;
    }

    bool Run(){
        if (!initialized){
            steps[0] -> runner -> Begin();
            initialized = true;
        }
        if (!done){
            totalFrames ++;
            if (/*(totalFrames > steps[stepNum] -> maxFrameCount && steps[stepNum] -> maxFrameCount > 0) || */(steps[stepNum] -> runner -> run())) { // When a step finishes.
                totalFrames = 0;
                steps[stepNum] -> runner -> Cleanup();
                stepNum ++;
                if (stepNum == steps.size()){ // POTENTIAL BUG: THIS MIGHT NEED A -1 SOMEWHERE!
                    done = true;
                    return true;
                }
                steps[stepNum] -> runner -> Begin(); // Never runs if it exits, as above.
            }
        }
        return false;
    }
};
#endif
