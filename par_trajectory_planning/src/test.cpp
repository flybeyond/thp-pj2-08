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

	std::cout << "connection success" << std::endl;

	struct timeval timeout_begin_old;
	struct timeval timeout_end_old;

	/* Save original timeout */
	modbus_get_timeout_begin(ctx, &timeout_begin_old);
	printf(" ---> timeout begin tv_sec %ld and timeout tv_usec %ld\n", timeout_begin_old.tv_sec, timeout_begin_old.tv_usec); 

	modbus_get_timeout_end(ctx, &timeout_end_old);
	printf(" ---> timeout end tv_sec %ld and timeout tv_usec %ld\n", timeout_end_old.tv_sec, timeout_end_old.tv_usec);

        modbus_set_slave(ctx, 1);
	printf("errno: %s\n", modbus_strerror(errno));


	// Set M[0]..M[5] to 0 else wrong position is selected 
	int n = modbus_write_register(ctx, 0x001E, 0x0000);
	printf("errno: %s\n", modbus_strerror(errno));

        //  turn ON the motor excitation
        n = modbus_write_register(ctx, 0x001E, 0x2000);
	printf("errno: %s\n", modbus_strerror(errno));


	uint16_t* src = new uint16_t[2];
	// POSITION NO1 is address 0x0402 and 0x0403 and specify position 1000
	src[1] = 0x03E8;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0402, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// POSITION NO1 set operating speed
	src[1] = 0x1388;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0502, 2, src);

	// POSITION NO1 set operating mode to link
	n = modbus_write_register(ctx, 0x0701, 0x01);

	// POSITION NO1 set to absolute positioning
	n = modbus_write_register(ctx, 0x0601, 0x01);

	// POSITION NO1 set to sequential mode
	n = modbus_write_register(ctx, 0x0801, 0x01);

	// POSITION NO2 is address 0x0404 and 0x045 and specify position
	src[1] = 0x1710;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0404, 2, src);

	// POSITION NO2 set operating speed
	src[1] = 0x0800;
	src[0] = 0x00;
	n = modbus_write_registers(ctx, 0x0504, 2, src);

	// POSITION NO2 set operating mode to single
	n = modbus_write_register(ctx, 0x0702, 0x00);

	// POSITION NO2 set to absolute positioning
	n = modbus_write_register(ctx, 0x0602, 0x01);

	// POSITION NO2 set to sequential mode
	n = modbus_write_register(ctx, 0x0802, 0x01);



	// turn start input on
	n = modbus_write_register(ctx, 0x001E, 0x2101);

	// turn start input off
	//n = modbus_write_register(ctx, 0x001E, 0x2000);

	modbus_close(ctx);
	modbus_free(ctx);


        return 0;

}
