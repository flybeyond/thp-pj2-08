#include <iostream>
#include <ros/ros.h>
#include <par_trajectory_planning/commands.h>
#include <par_trajectory_planning/steppermotor.h>
#include <par_trajectory_planning/config.h>
#include <cerrno>
//#include <modbus/modbus.h>
//#include <cstdio>


static const int QUEUE_SIZE = 1000;
static StepperMotor* stepperMotor;

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

/**
 * @brief This is the callback function responsible for receiving and processing commands from the user.
 * @param command object
 */
void commands(const par_trajectory_planning::commands& cmd)
{
    std::cout << "received option: " << cmd.option << std::endl;
    switch(cmd.option)
    {
        case MENU_INIT_COMM:
	    std::cout << "init communication" << std::endl;
            stepperMotor->initCom();
        break;
        case MENU_INIT_MOTOR: 
            stepperMotor->init();
        break;
        case MENU_CONF_SIN_MOT:
            stepperMotor->confSingleMotion(cmd);
        break;
        case MENU_CONF_PTP_MOT:
            stepperMotor->confPTPMotion(cmd);
        break;
        case MENU_START_MOT:
            stepperMotor->start();
        break;
        case MENU_EXIT:
            stepperMotor->exit();
        break;  
    }
}

int main(int argc, char **argv)
{
    std::cout << "par_trajectory_planning." << std::endl;
    //ros::init(argc, argv, "par_trajectory_planning", ros::init_options::NoSigintHandler);
    //ros::NodeHandle n;
    modbus_t* ctx = modbus_new_rtu("/dev/ttyS0", 115200, 'N', 8, 1); 
    //init_communication(ctx);
    //configure_PTP_motion(ctx);
    par_trajectory_planning::commands cmd;
    cmd.xyz_pos.clear();
    stepperMotor = new StepperMotor(ctx);
    stepperMotor->initCom();
    cmd.xyz_pos.push_back(-41.5733);
    cmd.xyz_pos.push_back(-41.5733);
    cmd.xyz_pos.push_back(-41.5733);
    stepperMotor->confPTPMotion(cmd);
    //stepperMotor->start();
    //init_communication(ctx);
    //configure_PTP_motion(ctx);
    
    //ros::Subscriber sub = n.subscribe("commands", QUEUE_SIZE, commands);

    //ros::spin();

    return 0;
}
