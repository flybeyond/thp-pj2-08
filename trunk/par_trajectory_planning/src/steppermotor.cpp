#include <par_trajectory_planning/steppermotor.h>
#include <par_trajectory_planning/config.h>
#include <cstdio>
#include <cmath>
#include <cerrno>

void StepperMotor::init()
{
    std::cout << "StepperMotor::init()" << std::endl;
    motions = 0;
}

void StepperMotor::start()
{
    std::cout << "StepperMotor::start()" << std::endl;
    uint16_t stat_ax01[2], stat_ax02[2], stat_ax03[3];
    int i, n;
    for(i=1; i<motions; i++)
    {
	    modbus_set_slave(ctx, 0);
        n = modbus_write_register(ctx, 0x001E, 0x2000);
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
    modbus_flush(ctx);
    printf("errno: %s\n", modbus_strerror(errno));
}

void StepperMotor::stop()
{
    std::cout << "StepperMotor::stop()" << std::endl;
}

void StepperMotor::initCom()
{
    std::cout << "StepperMotor::initCom()" << std::endl;
	if (ctx == NULL)
	{
		std::cout << "StepperMotor::init(): unable to create the libmodbus context." << std::endl;
	}
	
	modbus_set_debug(ctx, 1);
	struct timeval timeout_end;
	struct timeval timeout_begin;
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

void StepperMotor::confSingleMotion(const par_trajectory_planning::commands& cmd)
{
    std::cout << "StepperMotor::confSingleMotion()" << std::endl;

	std::cout << "vector size: " << cmd.abs_pos.size() << std::endl;
    motions = cmd.abs_pos.size() / 6;

    int i;
    for(i=0; i<motions; i++)
    {
        // pos up X
        // pos lo X
        initSingleMotion(MODBUS_SLAVE_ADDR_01, cmd.abs_pos[ 0 + (i * 6) ], cmd.abs_pos[ 1 + (i * 6) ], i + 1);    

        // pos up Y
        // pos lo Y
        initSingleMotion(MODBUS_SLAVE_ADDR_02, cmd.abs_pos[ 2 + (i * 6) ], cmd.abs_pos[ 3 + (i * 6) ], i + 1); 

        // pos up Z
        // pos lo Z
        initSingleMotion(MODBUS_SLAVE_ADDR_03, cmd.abs_pos[ 4 + (i * 6) ], cmd.abs_pos[ 5 + (i * 6) ], i + 1); 
    }

}

void StepperMotor::confPTPMotion(const par_trajectory_planning::commands& cmd)
{
    std::cout << "StepperMotor::confPTPMotion" << std::endl;
    motions = cmd.xyz_pos.size() / 3;
    
    int i;
    uint16_t pos_lo = 0;
    uint16_t pos_up = 0;
    for(i=0; i<motions; i++)
    {
        pos_lo = angleToStep( cmd.xyz_pos[0] );
        pos_up = ( pos_lo & 0x8000 ) ? 0xFFFF : 0x00;
        initSingleMotion(MODBUS_SLAVE_ADDR_01, pos_up, pos_lo, i + 1); // X
        
        pos_lo = angleToStep( cmd.xyz_pos[1] );
        pos_up = ( pos_lo & 0x8000 ) ? 0xFFFF : 0x00;
        initSingleMotion(MODBUS_SLAVE_ADDR_02, pos_up, pos_lo, i + 1); // Y
        
        pos_lo = angleToStep( cmd.xyz_pos[2] );
        pos_up = ( pos_lo & 0x8000 ) ? 0xFFFF : 0x00;
        initSingleMotion(MODBUS_SLAVE_ADDR_03, pos_up, pos_lo, i + 1); // Z
    }
}

void StepperMotor::exit()
{
    std::cout << "StepperMotor::exit()" << std::endl;
    if (ctx != NULL)
    {
	    modbus_close(ctx);
	    modbus_free(ctx);    
    }
}

void StepperMotor::initSingleMotion(int slave, uint16_t pos_lo, uint16_t pos_up, int off)
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

uint16_t StepperMotor::angleToStep(double x)
{
    return static_cast<uint16_t>(round((90.0 + x) / MOTOR_STEP_ANGLE));
}
