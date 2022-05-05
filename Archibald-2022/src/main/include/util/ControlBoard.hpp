/* By Tyler Clarke, any external contributions under any circumstances or for any reason whatsoever will be acknowledged here.
    Just your average neighborhood grand unified controller - Xbox and Button Board.
*/

#include <frc/XboxController.h>
#include <frc/GenericHID.h>

#include <constants.h>

#include <util/RunManager.hpp>


struct ControlButton{
    bool buttonState = false;
    bool buttonReleased = false;

    void Run(bool state){
        if (state){
            buttonState = true;
        }
        else if (buttonState){
            buttonReleased = true;
            buttonState = false;
        }
    }

    bool GetWasReleased(){
        if (buttonReleased){
            buttonReleased = false;
            return true;
        }
        return false;
    }
};


struct ControlBoard : Runnable{
    frc::XboxController* xbox;
    frc::GenericHID* buttonboard;

    double deadband = UNIVERSAL_DEADBAND;

    bool intakeMacroButtonState = false;
    bool intakeMacroButtonReleased = false;

    bool shootMacroButtonState = false;
    bool shootMacroButtonReleased = false;

    ControlButton intakeMacroButton;
    ControlButton intakeUpButton;
    ControlButton intakeDownButton;

    ControlButton shootMacroButton;

    ControlBoard(int xboxSlot = XBOX_CONTROLLER_SLOT, int buttonboardSlot = BUTTONBOARD_CONTROLLER_SLOT){
        xbox = new frc::XboxController(xboxSlot);
        buttonboard = new frc::GenericHID(buttonboardSlot);
    }

    ControlBoard(frc::XboxController* xboxController, frc::GenericHID* buttonboardController){
        xbox = xboxController;
        buttonboard = buttonboardController;
    }

    void SetDeadband(double band){
        deadband = band;
    }

    double GetLeftY(){
        double val = xbox -> GetRawAxis(XBOX_JOYSTICK_LEFT_Y);
        if (fabs(val) > deadband){
            return val;
        }
        return 0;
    }

    double GetRightY(){
        double val = xbox -> GetRawAxis(XBOX_JOYSTICK_RIGHT_Y);
        if (fabs(val) > deadband){
            return val;
        }
        return 0;
    }

    double GetLeftX(){
        double val = xbox -> GetRawAxis(XBOX_JOYSTICK_LEFT_X);
        if (fabs(val) > deadband){
            return val;
        }
        return 0;
    }

    double GetRightX(){
        double val = xbox -> GetRawAxis(XBOX_JOYSTICK_RIGHT_X);
        if (fabs(val) > deadband){
            return val;
        }
        return 0;
    }

    double GetSpeedLimit(){
        return (buttonboard -> GetRawAxis(BUTTONBOARD_AXIS_SPEEDLIMIT) + 1) / 2;
    }

    double GetShooterVariable(){
        return buttonboard -> GetRawAxis(BUTTONBOARD_SHOOTER_AXIS); // I hate the naming mismatch; will change when I delete the old code. Can't wait for that!
    }

    uint8_t GetShooterSpeedSwitch(){
        if (buttonboard -> GetRawButton(BUTTONBOARD_SHOOTERSPEED_LOW)){
            return 0; // 0 = low
        }
        else if (buttonboard -> GetRawButton(BUTTONBOARD_SHOOTERSPEED_VARIABLE)){
            return 2; // 2 = variable
        }
        else{
            return 1; // 1 = high, this is default.
        }
    }

    double GetShooterSpeed(){
        uint8_t shooterSwitchValue = GetShooterSpeedSwitch();
        if (shooterSwitchValue == 0){
            return SHOOTERSPEED_LOW;
        }
        else if (shooterSwitchValue == 1){
            return SHOOTERSPEED_HIGH;
        }
        else{
            return GetShooterVariable();
        }
    }

    bool GetIntakeMacroButtonState(){
        return buttonboard -> GetRawButton(BUTTONBOARD_MACRO_INTAKE);
    }

    bool GetShootMacroButtonState(){
        return buttonboard -> GetRawButton(BUTTONBOARD_MACRO_SHOOT);
    }

    bool GetIntakeUpButtonState(){
        return buttonboard -> GetRawButton(BUTTONBOARD_RAISE_INTAKE);
    }

    bool GetIntakeMacroButtonReleased(){
        return intakeMacroButton.GetWasReleased();
    }

    bool GetIntakeUpButtonReleased(){
        return intakeUpButton.GetWasReleased();
    }

    bool GetShootMacroButtonReleased(){
        return shootMacroButton.GetWasReleased();
    }

    void Run(){
        intakeMacroButton.Run(GetIntakeMacroButtonState());
        intakeUpButton.Run(GetIntakeUpButtonState());
        shootMacroButton.Run(GetShootMacroButtonState());
    }
};
