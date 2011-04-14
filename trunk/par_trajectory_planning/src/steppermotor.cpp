#include <par_trajectory_planning/steppermotor.h>
#include <cstdio>
#include <cmath>
#include <cerrno>

void StepperMotor::init()
{
    ctx = modbus_new_rtu("/dev/ttyS0", 115200, 'N', 8, 1);
    if (ctx == NULL)
    {
            std::cout << "StepperMotor::init(): unable to create the libmodbus context." << std::endl;
    }
    modbus_set_debug(ctx, 1);

    if ( modbus_connect(ctx) == -1)
    {
        std::cout << "StepperMotor::init(): connection failed." << std::endl;
    }
    
	/*
	 * Configuration necessary to put multiple controllers in one group
	 */
    modbus_set_slave(ctx, 0x01);
    int n = modbus_write_register(ctx, 0x0030, 0x01);    

}

bool StepperMotor::startPTPMotion(double x, double y, double z)
{
	/*
	 * Positioning operation (see page 75)
	 *
	 */
	printf("errno: %s\n", modbus_strerror(errno));
    // turn ON the motor excitation
    int n = modbus_write_register(ctx, 0x001E, 0x2000);
	printf("errno: %s\n", modbus_strerror(errno));

    // set motor position
	uint16_t* src = new uint16_t[2];
	src[1] = 0x03E8;
	src[0] = 0x00;	
	n = modbus_write_registers(ctx, 0x0402, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));
    
    // set operating speed
	src[1] = 0x1388;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0502, 2, src);
	
	// turn the START input ON (start operation)
	n = modbus_write_register(ctx, 0x001E, 0x2101);
	
	// turn the START input OFF
	n = modbus_write_register(ctx, 0x001E, 0x2001);
    
    return false;
}
