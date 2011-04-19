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


        modbus_set_slave(ctx, 0);
	printf("errno: %s\n", modbus_strerror(errno));

	//int n = modbus_write_register(ctx, 0x001E, 0x2002);

	uint16_t* dest = new uint16_t[1];
	//modbus_read_registers(ctx, 0x117, 1, dest); 
	//std::cout << "0: present selected operation data number: " << dest[0] << std::endl;
    	//  turn ON the motor excitation
    	int n = modbus_write_register(ctx, 0x001E, 0x2000);
	printf("errno: %s\n", modbus_strerror(errno));
	// turn start input on
	n = modbus_write_register(ctx, 0x001E, 0x2101);
	printf("errno: %s\n", modbus_strerror(errno));
	sleep(1);
	// turn start input off
	n = modbus_write_register(ctx, 0x001E, 0x2001);
	printf("errno: %s\n", modbus_strerror(errno));

	uint16_t* status = new uint16_t[1];
	//status[0] = 0x00;
	//status[1] = 0x00;
	//modbus_read_registers(ctx, 0x0020, 2, status); 
	//printf("motor status READY [0, 1]: %x\n", (status[0] & 0x2000));

	// do nothing	
	//while( (status[0] & 0x2000) == 0 )
	//{
	//	modbus_read_registers(ctx, 0x0020, 2, status); 
	//}

	delete(dest);
	modbus_close(ctx);
	modbus_free(ctx);


        return 0;

}
