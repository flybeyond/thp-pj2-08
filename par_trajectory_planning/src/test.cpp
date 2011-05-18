#include <modbus/modbus.h>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <cerrno>

const uint16_t MOTOR_SPEED_UP  = 0x1388;
const uint16_t MOTOR_SPEED_LO  = 0x0000;

const uint16_t REG_MOTOR_POS   = 0x0400; // 2 bytes per offset 
const uint16_t REG_MOTOR_SPEED = 0x0500; // 2 bytes per offset
const uint16_t REG_MOTOR_ABS   = 0x0600; // 1 byte per offset

void configure_motion(modbus_t* ctx, int slave, uint16_t pos_up, uint16_t pos_lo, int off)
{
	uint16_t src[2];
	// POSITION NO1 is address 0x0402 and 0x0403 and specify position pos_up / pos_lo
	modbus_set_slave(ctx, slave);
	src[1] = pos_up;
	src[0] = pos_lo;
	int n = modbus_write_registers(ctx, REG_MOTOR_POS + (off * 2), 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set operating speed
	src[1] = MOTOR_SPEED_UP;
	src[0] = MOTOR_SPEED_LO;
	n = modbus_write_registers(ctx, REG_MOTOR_SPEED + (off * 2), 2, src);
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set to absolute positioning
	n = modbus_write_register(ctx, REG_MOTOR_ABS + (off * 1), 0x01);	
	printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set operating mode to single motion
	//n = modbus_write_register(ctx, 0x0701, 0x00);
	//printf("errno: %s\n", modbus_strerror(errno));

	// 1POSITION NO1 set to sequential mode
	//n = modbus_write_register(ctx, 0x0801, 0x00);
	//printf("errno: %s\n", modbus_strerror(errno));

	// ugly
	usleep(10000);
} 

void execute_motion(modbus_t* ctx, int off)
{
        modbus_set_slave(ctx, 0);
	printf("errno: %s\n", modbus_strerror(errno));

	//int n = modbus_write_register(ctx, 0x001E, 0x2002);
	//modbus_read_registers(ctx, 0x117, 1, dest); 
	//std::cout << "0: present selected operation data number: " << dest[0] << std::endl;
    //  turn ON the motor excitation

    int n = modbus_write_register(ctx, 0x001E, 0x2000);
	printf("errno: %s\n", modbus_strerror(errno));
	// turn start input on
	n = modbus_write_register(ctx, 0x001E, 0x2100 + (off) );
	printf("errno: %s\n", modbus_strerror(errno));
	// turn start input off
	n = modbus_write_register(ctx, 0x001E, 0x2000 + (off) );
	printf("errno: %s\n", modbus_strerror(errno));
}

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
	//timeout.tv_usec =  500000;
	//modbus_set_timeout_begin(ctx, &timeout);


        if ( modbus_connect(ctx) == -1)
        {
                std::cout << "StepperMotor::init(): connection failed." << std::endl;
        }



	//
        // execute the position crap
        //
	int n = 1;
	uint16_t pos_up = 0x0000;
	uint16_t pos_lo = 0x0000;
	while(1)
	{
		std::cout << "configure motion (enter 1 to exit): " << std::endl;
		std::cout << "pos up: ";
		std::cin >> pos_up;
		if (pos_up == 1) break;
		std::cout << "pos lo: ";
		std::cin >> pos_lo;

		configure_motion(ctx, 1, pos_up, pos_lo, n);
		configure_motion(ctx, 2, pos_up, pos_lo, n);
		configure_motion(ctx, 3, pos_up, pos_lo, n);
		n++;
	}

	int i = 1;
	while(i < n)
	{
		execute_motion(ctx, i);
		i++;
	}

	//status monitoring
	//uint16_t* status = new uint16_t[1];
	//status[0] = 0x00;
	//status[1] = 0x00;
	//modbus_read_registers(ctx, 0x0020, 2, status); 
	//printf("motor status READY [0, 1]: %x\n", (status[0] & 0x2000));
	// do nothing	
	//while( (status[0] & 0x2000) == 0 )
	//{
	//	modbus_read_registers(ctx, 0x0020, 2, status); 
	//}
	//delete(dest);
	modbus_close(ctx);
	modbus_free(ctx);


        return 0;

}