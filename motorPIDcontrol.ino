#include "motorPIDcontrol.h"

    
void motorPIDcontrolSetup()
{
    *controlledMotors = ControlledMotors(motor1, motor2, encoder0, encoder1);
}
