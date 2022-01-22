// Lots of FRC/Wpi legalese crap to dredge through, but this is the 21st century and when we see a smelly swamp we build things on it.
/*





You see how nice things can be built upon the swamps of copyright?
                                             ________________________________________________
                                            |                                                |
                                            |   ________                          ________   |
                                            |  |        |                        |        |  |
                                            |  |   n    |                        |   Y    |  |
                                            |  |   |    |                        |   |    |  |
                                            |  |  \_]   |                        |  \_]   |  |
                                            |  |mmmmmmmm|                        |mmmmmmmm|  |
                                            |                                                |
                                            |                                                |
                                            |   ________                          ________   |
                                            |  |        |                        |   V    |  |
                                            |  |   U    |                        |  FVF   |  |
                                            |  |   |    |                        |   |    |  |
                                            |  |  \_]   |                        |  \_]   |  |
                                            |  |mmmmmmmm|                        |mmmmmmmm|  |
                                            |                                                |
                                            |                                                |
                                            |   ________                          ________   |
                                            |  |        |                        |        |  |
                                            |  |  ooo   |                        |   0    |  |
                                            |  |   |    |                        |   |    |  |
                                            |  |  \_]   |                        |  \_]   |  |
                                            |  |mmmmmmmm|                        |mmmmmmmm|  |
                                            |                                                |
                                            |                                                |
                                            |   ________                          ________   |
                                            |  |  zZz   |                        |  XX    |  |
                                            |  |  zZz   |                        |   XX   |  |
                                            |  |   |    |                        |   |    |  |
                                            |  |  \_]   |                        |  \_]   |  |
                                            |  |mmmmmmmm|        _______         |mmmmmmmm|  |
                                            |                   |       |                    |
                                            |                   |       |                    |
                                            |                   |       |                    |
                                            |                   |     O |                    |
                                            |                   |       |                    |
                                            |                   |       |                    |
 ___________________________________________--------------------------------------------------_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
|                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|__________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________|
Distribution includes (but is not possible to put the Notice in a larger work of which you may at your option offer warranty protection in exchange for a fee. You may always continue to use the Work by You to the interfaces of, the Licensor except as expressly stated in Sections 2(a) and 2(b) above, Recipient receives no rights or licenses to their respective portions thereof. Deploy" means: (a) to sublicense, distribute or transfer NetHack except as disclosed pursuant to Section 3.4(a) above,
Contributor believes that Contributor's Modifications are derived, directly or indirectly infringes any patent where such claim is resolved (such as a whole.

If identifiable sections of this License, shall survive. Termination Upon Assertion of Patent Infringement. If you do not forfeit any of the original test modes be preserved. If you make it clear that any such warranty or additional liability.

*/

#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>
#include <ctre/Phoenix.h>
#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>
#include <frc/Notifier.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <stdlib.h>
#include <stdio.h>
#include <atomic>


#include "ModularRobot.hpp"


class Robot : public ModularRobot{
private:
    std::atomic<bool> m_exit{false}; // Multithreaded variable. This is why the code doesn't die!

public:

    Robot(){
        setData("Archibald", "Firestorm Robotics", 6341);
    }

    static void Periodic(Robot *self){

    }

    void Init(){

    }

    void BeginTeleop(){

    }

    void TeleopLoop(){

    }

    void TeleopPeriodic(){

    }

    void TestLoop(){

    }

    void BeginTest(){

    }

    void AutonomousLoop(){

    }

    void CleanUpTest(){

    }

    void CleanUpTeleop(){

    }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
