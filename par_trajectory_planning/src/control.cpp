// compile instruction 
// g++ control.cpp -o control -lboost_signals -lboost_system -lmodbus

#include <iostream>
#include <cstdio>
#include <cmath>
#include <cerrno>
#include <modbus/modbus.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>

const int MENU_INIT_COMM    = 1;
const int MENU_INIT_MOTOR   = 2;
const int MENU_CONF_SIN_MOT = 3;
const int MENU_CONF_PTP_MOT = 4;
const int MENU_START_MOT    = 5;
const int MENU_EXIT         = 0;

const int MODBUS_MAX_PROC_TIME  = 4000; // in usec
const int MODBUS_MAX_BCAST_TIME = 7500; // in usec
const int MODBUS_TIMEOUT_BEGIN  = 50000; // in usec
const int MODBUS_TIMEOUT_END    = 50000;
const int MODBUS_SLAVE_ADDR_01  = 0x01;
const int MODBUS_SLAVE_ADDR_02  = 0x02;
const int MODBUS_SLAVE_ADDR_03  = 0x03;

const uint16_t MOTOR_SPEED_UP  = 0x4000;
const uint16_t MOTOR_SPEED_LO  = 0x0000;

const uint16_t REG_MOTOR_POS   = 0x0400; // 2 bytes per offset 
const uint16_t REG_MOTOR_SPEED = 0x0500; // 2 bytes per offset
const uint16_t REG_MOTOR_ABS   = 0x0600; // 1 byte per offset


int menu()
{
    std::cout << "[" << MENU_INIT_COMM << "] init communication" << std::endl;
    std::cout << "[" << MENU_INIT_MOTOR << "] init motors" << std::endl;
    std::cout << "[" << MENU_CONF_SIN_MOT << "] configure single motion" << std::endl;
    std::cout << "[" << MENU_CONF_PTP_MOT << "] configure PTP motion" << std::endl;
    std::cout << "[" << MENU_START_MOT << "] start motion" << std::endl;
    std::cout << "[" << MENU_EXIT << "] exit" << std::endl;
    
    int x = 0;
    std::cin >> x;
    return x;
}

void init_motor()
{
}

void init_communication(modbus_t* ctx)
{

	//ctx = modbus_new_rtu("/dev/ttyS0", 115200, 'N', 8, 1);
	if (ctx == NULL)
	{
		std::cout << "StepperMotor::init(): unable to create the libmodbus context." << std::endl;
	}
	modbus_set_debug(ctx, 1);
	struct timeval timeout_end;
	struct timeval timeout_begin;
	//timeout.tv_sec = 0;
	//timeout.tv_usec =  500000;
	//modbus_set_timeout_begin(ctx, &timeout);
	modbus_get_timeout_end(ctx, &timeout_end);
	printf("timeout.tv_sec: %ld\n", timeout_end.tv_sec);
	printf("timeout.tv_usec: %ld\n", timeout_end.tv_usec);
	timeout_end.tv_usec = MODBUS_TIMEOUT_END;
	modbus_set_timeout_end(ctx, &timeout_end);

	modbus_get_timeout_begin(ctx, &timeout_begin);
	printf("timeout.tv_sec: %ld\n", timeout_begin.tv_sec);
	printf("timeout.tv_usec: %ld\n", timeout_begin.tv_usec);
	timeout_begin.tv_usec = MODBUS_TIMEOUT_BEGIN;
	modbus_set_timeout_begin(ctx, &timeout_begin);

	if ( modbus_connect(ctx) == -1)
	{
		std::cout << "StepperMotor::init(): connection failed." << std::endl;
	}	
}

void start_motion(modbus_t* ctx, int motions)
{
    uint16_t stat_ax01[2], stat_ax02[2], stat_ax03[3];
    int i;
    for(i=1; i<motions; i++)
    {
	modbus_set_slave(ctx, 0);
        int n = modbus_write_register(ctx, 0x001E, 0x2000);
	printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_BCAST_TIME);
	// turn start input on
	n = modbus_write_register(ctx, 0x001E, 0x2100 + i );
	printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_BCAST_TIME);
	// turn start input off
	n = modbus_write_register(ctx, 0x001E, 0x2000 + i );
	printf("errno: %s\n", modbus_strerror(errno));    
	usleep(MODBUS_MAX_BCAST_TIME);

	stat_ax01[0] = stat_ax02[0] = stat_ax03[0] = 0x0000;
	stat_ax01[1] = stat_ax02[1] = stat_ax03[1] = 0x0000;
	while( (stat_ax01[0] & 0x2000) == 0 ||
	       (stat_ax02[0] && 0x2000) == 0 ||
	       (stat_ax03[0] && 0x2000) == 0 )
	{
		modbus_set_slave(ctx, MODBUS_SLAVE_ADDR_01);
		modbus_read_registers(ctx, 0x0020, 2, stat_ax01); 
		usleep(MODBUS_MAX_PROC_TIME);

		modbus_set_slave(ctx, MODBUS_SLAVE_ADDR_02);
		modbus_read_registers(ctx, 0x0020, 2, stat_ax02); 
		usleep(MODBUS_MAX_PROC_TIME);

		modbus_set_slave(ctx, MODBUS_SLAVE_ADDR_03);
		modbus_read_registers(ctx, 0x0020, 2, stat_ax03); 
		usleep(MODBUS_MAX_PROC_TIME);
		// this is zero if the motor is moving
	}
	std::cout << "motor can receive new command\n" << std::endl;
    }  
}

void configure_single_motion(modbus_t* ctx, int slave, uint16_t pos_lo, uint16_t pos_up, int off)
{
	/*if (slave == 1)
	{
		src[0] = 0;
		src[1] = 0;
		n = modbus_read_registers(ctx, REG_MOTOR_POS + (off * 2), 2, src);
		usleep(MODBUS_MAX_PROC_TIME);
		printf("pos_up: %x\n", src[1]);
		printf("pos_lo: %x\n", src[0]);
	}*/
	
	uint16_t src[2];
	// position is address 0x0402 and 0x0403 and specify position pos_up / pos_lo
	modbus_set_slave(ctx, slave);

	src[1] = pos_up;
	src[0] = pos_lo;
	int n = modbus_write_registers(ctx, REG_MOTOR_POS + (off * 2), 2, src);
	printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_PROC_TIME);
	
	// set operating speed
	src[1] = MOTOR_SPEED_UP;
	src[0] = MOTOR_SPEED_LO;
	n = modbus_write_registers(ctx, REG_MOTOR_SPEED + (off * 2), 2, src);
	printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_PROC_TIME);

	// set to absolute positioning
	n = modbus_write_register(ctx, REG_MOTOR_ABS + (off * 1), 0x01);	
	printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_PROC_TIME);
}

int configure_PTP_motion(modbus_t* ctx)
{
    std::cout << "Please enter the amount of motions you want to configure: " << std::endl;
    int motions = 0;
    std::cin >> motions;

    uint16_t pos_up[3];
    uint16_t pos_lo[3];   
    int i = 1;
    while(i <= motions)
    {
		//std::cout << "pos up for axis 1: ";
		//std::cin >> pos_up[0]; 
		pos_up[0] = 0;
		std::cout << "pos lo for axis 1: ";
		std::cin >> pos_lo[0];
		if (pos_lo[0] & 0x8000) pos_up[0] = 0xFFFF;

		//std::cout << "pos up for axis 2: ";
		//std::cin >> pos_up[1]; 
		pos_up[1] = 0;
		std::cout << "pos lo for axis 2: ";
		std::cin >> pos_lo[1];
		if (pos_lo[1] & 0x8000) pos_up[1] = 0xFFFF;

		//std::cout << "pos up for axis 3: ";
		//std::cin >> pos_up[2]; 
		pos_up[2] = 0;
		std::cout << "pos lo for axis 3: ";
		std::cin >> pos_lo[2];
		if (pos_lo[2] & 0x8000) pos_up[2] = 0xFFFF;
		
		configure_single_motion(ctx, MODBUS_SLAVE_ADDR_01, pos_up[0], pos_lo[0], i);
		configure_single_motion(ctx, MODBUS_SLAVE_ADDR_02, pos_up[1], pos_lo[1], i);
		configure_single_motion(ctx, MODBUS_SLAVE_ADDR_03, pos_up[2], pos_lo[2], i);
		
        	i++;
    }
    
    return i;
}

int main()
{
	modbus_t* ctx = modbus_new_rtu("/dev/ttyS0", 115200, 'N', 8, 1);
	/*

	ctx = modbus_new_rtu("/dev/ttyS0", 115200, 'N', 8, 1);
	if (ctx == NULL)
	{
		std::cout << "StepperMotor::init(): unable to create the libmodbus context." << std::endl;
	}
	modbus_set_debug(ctx, 1);
	struct timeval timeout_end;
	struct timeval timeout_begin;
	//timeout.tv_sec = 0;
	//timeout.tv_usec =  500000;
	//modbus_set_timeout_begin(ctx, &timeout);
	modbus_get_timeout_end(ctx, &timeout_end);
	printf("timeout.tv_sec: %ld\n", timeout_end.tv_sec);
	printf("timeout.tv_usec: %ld\n", timeout_end.tv_usec);
	timeout_end.tv_usec = MODBUS_TIMEOUT_END;
	modbus_set_timeout_end(ctx, &timeout_end);

	modbus_get_timeout_begin(ctx, &timeout_begin);
	printf("timeout.tv_sec: %ld\n", timeout_begin.tv_sec);
	printf("timeout.tv_usec: %ld\n", timeout_begin.tv_usec);
	timeout_begin.tv_usec = MODBUS_TIMEOUT_BEGIN;
	modbus_set_timeout_begin(ctx, &timeout_begin);

	if ( modbus_connect(ctx) == -1)
	{
		std::cout << "StepperMotor::init(): connection failed." << std::endl;
	}*/

    int m = 0;
    int x = 1;
    
    while(x)
    {
        x = menu();
        switch(x)
        {
            case MENU_INIT_COMM:
                init_communication(ctx);
	    break;
            case MENU_INIT_MOTOR: 
                init_motor();
            break;
            case MENU_CONF_SIN_MOT:
            break;
            case MENU_CONF_PTP_MOT:
                m = configure_PTP_motion(ctx);
            break;
            case MENU_START_MOT:
                start_motion(ctx, m);
            case MENU_EXIT:
            break;
        }
    }
    
    if (ctx != NULL)
    {
	    modbus_close(ctx);
	    modbus_free(ctx);    
    }
    
    return 0;
}


