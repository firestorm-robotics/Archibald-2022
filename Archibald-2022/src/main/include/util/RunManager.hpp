/* By Tyler Clarke, any external contributions by anyone under any conditions will be acreditted here.
    Manages objects that comply with the Runnable spec.
*/
#ifndef RUNMANAGER_HPP
#define RUNMANAGER_HPP
#include <vector>


struct Runnable{
    Runnable(){}
    virtual void Run(){}
};


struct RunManager{ /* Allows you to group objects that support the Runnable spec and run them in one command, cleaning up code and making debugging less pleasant */
    std::vector<Runnable*> objects;
    void AddRunObject(Runnable* runnable){
        objects.push_back(runnable);
    }

    void RunAll(){
        for (unsigned int x = 0; x < objects.size(); x ++){
            objects[x] -> Run();
        }
    }
};
#endif
