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
    modbus_set_slave(ctx, 1);
    // addr 0316h, communication axis number, default value 15
    uint8_t* dest = new uint8_t[8];
    modbus_read_input_bits(ctx, 0x316, 8, dest);
    int i;
    for(i=0;i<8;i++)
    {
        std::cout << "dest[" << i << "]" << " := " << dest[i] << std::endl;
    }
    
    delete(dest);
    
    return false;
}
