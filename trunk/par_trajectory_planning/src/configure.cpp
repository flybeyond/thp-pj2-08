#include <modbus/modbus.h>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <cerrno>	

int main()
{
        modbus_t* ctx;

        ctx = modbus_new_rtu("/dev/ttyS0", 115200, 'N', 8, 1);
        if (ctx == NULL)
        {
                std::cout << "StepperMotor::init(): unable to create the libmodbus context." << std::endl;
        }
    	modbus_set_debug(ctx, 1);
	//struct timeval timeout;
	//timeout.tv_sec = 0;
	//timeout.tv_usec =  10000;
	//modbus_set_timeout_begin(ctx, &timeout);


        if ( modbus_connect(ctx) == -1)
        {
                std::cout << "StepperMotor::init(): connection failed." << std::endl;
        }
	//modbus_set_slave(ctx, 2);
	//modbus_write_register(ctx, 0x0030, 1);

	//modbus_set_slave(ctx, 3);
	//modbus_write_register(ctx, 0x0030, 1);


	uint16_t* src = new uint16_t[2];
	//
	 // AXIS 1
	 //
	// 
	// POSITION NO1 is address 0x0402 and 0x0403 and specify position 1000
	modbus_set_slave(ctx, 1);
	src[1] = 0x1388;
	src[0] = 0x00;
	int n = modbus_write_registers(ctx, 0x0402, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set operating speed
	src[1] = 0x1388;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0502, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set operating mode to single motion
	//n = modbus_write_register(ctx, 0x0701, 0x00);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set to absolute positioning
	//n = modbus_write_register(ctx, 0x0601, 0x01);	
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set to sequential mode
	//n = modbus_write_register(ctx, 0x0801, 0x00);
	printf("errno: %s\n", modbus_strerror(errno));

	sleep(1);

	//
	 // AXIS 2
	 //
	// 
	modbus_set_slave(ctx, 2);
	// POSITION NO1 is address 0x0402 and 0x0403 and specify position 1000
	src[1] = 0x1388;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0402, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set operating speed
	src[1] = 0x1388;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0502, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set operating mode to single motion
	//n = modbus_write_register(ctx, 0x0701, 0x00);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set to absolute positioning
	//n = modbus_write_register(ctx, 0x0601, 0x01);	
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set to sequential mode
	//n = modbus_write_register(ctx, 0x0801, 0x00);
	printf("errno: %s\n", modbus_strerror(errno));

	sleep(1);


	//
	 // AXIS 3
	 //
	// 
	modbus_set_slave(ctx, 3);
	// POSITION NO1 is address 0x0402 and 0x0403 and specify position 1000
	src[1] = 0x1388;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0402, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set operating speed
	src[1] = 0x1388;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0502, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set operating mode to single motion
	//n = modbus_write_register(ctx, 0x0701, 0x00);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set to absolute positioning
	//n = modbus_write_register(ctx, 0x0601, 0x01);	
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set to sequential mode
	//n = modbus_write_register(ctx, 0x0801, 0x00);
	printf("errno: %s\n", modbus_strerror(errno));




	/*	
	// 2POSITION NO2 is address 0x0404 and 0x0405 and specify position
	src[1] = 0x2710;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0404, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 2POSITION NO2 set operating speed
	src[1] = 0x0800;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0504, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 2POSITION NO2 set operating mode to single motion
	n = modbus_write_register(ctx, 0x0702, 0x00);
	printf("errno: %s\n", modbus_strerror(errno));

	// 2POSITION NO2 set to absolute positioning
	n = modbus_write_register(ctx, 0x0602, 0x01);
	printf("errno: %s\n", modbus_strerror(errno));

	// 2POSITION NO2 set to sequential mode
	n = modbus_write_register(ctx, 0x0802, 0x01);
	printf("errno: %s\n", modbus_strerror(errno));


	// 3POSITION NO3 is address 0x0406 and 0x047 and specify position
	src[1] = 0x3A98;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0406, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 3POSITION NO3 set operating speed
	src[1] = 0x0800;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0506, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 3POSITION NO3 set operating mode to single motion
	n = modbus_write_register(ctx, 0x0703, 0x00);
	printf("errno: %s\n", modbus_strerror(errno));

	// 3POSITION NO3 set to absolute positioning
	n = modbus_write_register(ctx, 0x0603, 0x01);
	printf("errno: %s\n", modbus_strerror(errno));

	// 3POSITION NO3 set to sequential mode
	n = modbus_write_register(ctx, 0x0803, 0x01);
	printf("errno: %s\n", modbus_strerror(errno));
*/

	delete(src);
	modbus_close(ctx);
	modbus_free(ctx);

	return 0;
}
