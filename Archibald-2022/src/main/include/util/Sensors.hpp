/* By Tyler Clarke. Any other contributions in any circumstances whatsoever will be noted here.
    Does sensors stuff.
*/

#include <frc/DigitalInput.h>

struct DigitalButtonSensor {
    frc::DigitalInput input;

    bool toggleState = false;

    unsigned long presses = 0;

    DigitalButtonSensor(int port) : input(port) {

    }

    void Run() {
        if (input.Get() && !toggleState){
            toggleState = true;
        }
        else if (!input.Get()){
            toggleState = false;
            presses ++;
            pressed = true;
        }
    }

    bool WasPressed(){
        if (pressed){
            pressed = false;
            return true;
        }
        else {
            return false;
        }
    }
}
