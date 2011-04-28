// compile instruction 
// g++ control.cpp -o control -lboost_signals -lboost_system -lmodbus

#include <iostream>
#include <cstdio>
#include <cmath>
#include <cerrno>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <modbus/modbus.h>

const int MENU_INIT_COMM    = 1;
const int MENU_INIT_MOTOR   = 2;
const int MENU_CONF_SIN_MOT = 3;
const int MENU_CONF_PTP_MOT = 4;
const int MENU_START_MOT    = 5;
const int MENU_EXIT         = 0;

const int MODBUS_MAX_PROC_TIME  = 4000; // in usec
const int MODBUS_MAX_BCAST_TIME = 7500; // in usec
const int MODBUS_SLAVE_ADDR_01  = 0x01;
const int MODBUS_SLAVE_ADDR_02  = 0x02;
const int MODBUS_SLAVE_ADDR_03  = 0x03;

const uint16_t MOTOR_SPEED_UP  = 0x1388;
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
}

void start_motion(modbus_t* ctx, int motions)
{
    modbus_set_slave(ctx, 0);
    int i;
    for(i=1; i<motions; i++)
    {
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
    }  
}

void configure_single_motion(modbus_t* ctx, int slave, uint16_t pos_up, uint16_t pos_lo, int off)
{
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
    
	uint16_t pos_up = 0x0000;
	uint16_t pos_lo = 0x0000;    
    int i = 1;
    while(i <= motions)
    {
		std::cout << "pos up: ";
		std::cin >> pos_up;
		std::cout << "pos lo: ";
		std::cin >> pos_lo;
		configure_single_motion(ctx, MODBUS_SLAVE_ADDR_01, pos_up, pos_lo, i);
		configure_single_motion(ctx, MODBUS_SLAVE_ADDR_02, pos_up, pos_lo, i);
		configure_single_motion(ctx, MODBUS_SLAVE_ADDR_03, pos_up, pos_lo, i);
        i++;
    }
    
    return i;
}

int main()
{
    modbus_t* ctx = NULL;	
	int m = 0;
    int x = 1;
    
    while(x)
    {
        x = menu();
        switch(x)
        {
            case MENU_INIT_COMM:
                init_communication(ctx);
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


