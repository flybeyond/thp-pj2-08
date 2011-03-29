#include <par_trajectory_planning/steppermotor.h>

void StepperMotor::init()
{
    ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
    if (ctx == NULL)
    {
        std::cout << "StepperMotor::init(): unable to create the libmodbus context." << std::endl;
        return;
    }
    
    if ( modbus_connect(ctx) == -1)
    {
        std::cout << "StepperMotor::init(): connection failed." << std::endl;
        return;
    }
}

bool StepperMotor::startPTPMotion(double x, double y, double z)
{
    // send some modbus commands :)
    return false;
}
