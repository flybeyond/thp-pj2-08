/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 */

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
    int i, n, j;
    
    j = 0;
    std::cout << "REPEAT MOTIONS: " << repeat_motions << std::endl;
    std::cout << "MOTIONS: " << motions << std::endl;

    while(j < repeat_motions && !stop_motion)
    {
        for(i=1; i<=motions && !stop_motion; i++)
        {
	        modbus_set_slave(ctx, 0);
            n = modbus_write_register(ctx, 0x001E, 0x2000);
	        //printf("errno: %s\n", modbus_strerror(errno));
		    //std::cout << "--> [1] write result: " << n << std::endl;
	        usleep(MODBUS_MAX_BCAST_TIME);
	        // turn start input on
	        n = modbus_write_register(ctx, 0x001E, 0x2100 /*+ i*/ );
	        //printf("errno: %s\n", modbus_strerror(errno));
		    //std::cout << "--> [2] write result: " << n << std::endl;
	        usleep(MODBUS_MAX_BCAST_TIME);
	        // turn start input off
	        n = modbus_write_register(ctx, 0x001E, 0x2000 /*+ i*/ );
		    //std::cout << "--> [3] write result: " << n << std::endl;
	        //printf("errno: %s\n", modbus_strerror(errno));    
	        usleep(MODBUS_MAX_BCAST_TIME);
	

	        stat_ax01[0] = stat_ax02[0] = stat_ax03[0] = 0x0000;
	        stat_ax01[1] = stat_ax02[1] = stat_ax03[1] = 0x0000;
	        while( (stat_ax01[0] & 0x2000) == 0 ||
	               (stat_ax02[0] && 0x2000) == 0 ||
	               (stat_ax03[0] && 0x2000) == 0 )
	        {
		        modbus_set_slave(ctx, MODBUS_SLAVE_ADDR_01);
		        n = modbus_read_registers(ctx, 0x0020, 2, stat_ax01); 
		        usleep(MODBUS_MAX_PROC_TIME);

		        modbus_set_slave(ctx, MODBUS_SLAVE_ADDR_02);
		        n = modbus_read_registers(ctx, 0x0020, 2, stat_ax02); 
		        usleep(MODBUS_MAX_PROC_TIME);

		        modbus_set_slave(ctx, MODBUS_SLAVE_ADDR_03);
		        n = modbus_read_registers(ctx, 0x0020, 2, stat_ax03); 
		        usleep(MODBUS_MAX_PROC_TIME);
	        }
	        //std::cout << "motor can receive new command\n" << std::endl;
        }  
        j++;
    }
    stop_motion = false;
}

void StepperMotor::stop()
{
    stop_motion = true;
}

void StepperMotor::initCom()
{	
	//modbus_set_debug(ctx, 1);
	struct timeval timeout_end;
	struct timeval timeout_begin;
	modbus_get_timeout_end(ctx, &timeout_end);
	timeout_end.tv_usec = MODBUS_TIMEOUT_END;
	modbus_set_timeout_end(ctx, &timeout_end);

	modbus_get_timeout_begin(ctx, &timeout_begin);
	timeout_begin.tv_usec = MODBUS_TIMEOUT_BEGIN;
	modbus_set_timeout_begin(ctx, &timeout_begin);

	if ( modbus_connect(ctx) == -1)
	{
		std::cout << "StepperMotor::init(): connection failed." << std::endl;
	}
}

void StepperMotor::confSingleMotion(const par_trajectory_planning::commands& cmd)
{
    motions = cmd.abs_pos.size() / 6; 
    repeat_motions = 1;

    int i;
    for(i=0; i<motions; i++)
    {
        initSingleMotion(MODBUS_SLAVE_ADDR_01, cmd.abs_pos[ 0 + (i * 6) ], cmd.abs_pos[ 1 + (i * 6) ], 
            MOTOR_ACC_UP, MOTOR_ACC_LO, MOTOR_DEC_UP, MOTOR_DEC_LO, 
            MOTOR_OP_SPEED_UP, MOTOR_OP_SPEED_LO, MOTOR_OPM_SINGLE, 0x00, i + 1); // X

        initSingleMotion(MODBUS_SLAVE_ADDR_02, cmd.abs_pos[ 2 + (i * 6) ], cmd.abs_pos[ 3 + (i * 6) ],
            MOTOR_ACC_UP, MOTOR_ACC_LO, MOTOR_DEC_UP, MOTOR_DEC_LO, 
            MOTOR_OP_SPEED_UP, MOTOR_OP_SPEED_LO, MOTOR_OPM_SINGLE, 0x00, i + 1); // Y

        initSingleMotion(MODBUS_SLAVE_ADDR_03, cmd.abs_pos[ 4 + (i * 6) ], cmd.abs_pos[ 5 + (i * 6) ], 
            MOTOR_ACC_UP, MOTOR_ACC_LO, MOTOR_DEC_UP, MOTOR_DEC_LO,
            MOTOR_OP_SPEED_UP, MOTOR_OP_SPEED_LO, MOTOR_OPM_SINGLE, 0x00, i + 1); // Z
    }

}

void StepperMotor::confPTPMotion(const par_trajectory_planning::commands& cmd)
{
    motions = cmd.xyz_pos.size() / 3;
    repeat_motions = cmd.repeat_motions;
    if (repeat_motions == 0) repeat_motions = 1;    

    uint16_t src[2];
    modbus_set_slave(ctx, 0);
	src[1] = cmd.st_speed_lo;
	src[0] = cmd.st_speed_up;
	modbus_write_registers(ctx, REG_MOTOR_ST_SPEED, 2, src);
	printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_PROC_TIME);    

    int i;
    int linked_motions = 0;
    uint16_t seq_pos = 0x01;
    uint16_t pos_lo[3];
    uint16_t pos_up[3];
    bool invalid_motion[3] = {false, false, false};
    for(i=0; i<motions; i++)  
    {
        std::cout << " x : " << cmd.xyz_pos[ 0 + (i * 3) ] << std::endl;
        std::cout << " y : " << cmd.xyz_pos[ 1 + (i * 3) ] << std::endl;
        std::cout << " z : " << cmd.xyz_pos[ 2 + (i * 3) ] << std::endl;
        pos_lo[X] = angleToStep( cmd.xyz_pos[ 0 + (i * 3) ], invalid_motion[X] );
        pos_up[X] = ( pos_lo[X] & 0x8000 ) ? 0xFFFF : 0x00;
        
        pos_lo[Y] = angleToStep( cmd.xyz_pos[ 1 + (i * 3) ], invalid_motion[Y] );
        pos_up[Y] = ( pos_lo[Y] & 0x8000 ) ? 0xFFFF : 0x00;
        
        pos_lo[Z] = angleToStep( cmd.xyz_pos[ 2 + (i * 3) ], invalid_motion[Z] );
        pos_up[Z] = ( pos_lo[Z] & 0x8000 ) ? 0xFFFF : 0x00;

        // disable sequential positioning for last entry.
        seq_pos = (i == (motions - 1)) ? 0x00 : 0x01;
        if (i == (motions - 1)) std::cout << "SEQUENTIAL POSITIONING DISABLED FOR DATA NO. " << i << std::endl;
        uint16_t operating_mode = cmd.operating_mode[i];

	    if (invalid_motion[X] ||
                invalid_motion[Y] ||
                invalid_motion[Z] )
	    {
		    std::cout << "INVALID MOTION; NOT CONFIGURED!" << std::endl;
	    }
	    else
	    {
		    initSingleMotion(MODBUS_SLAVE_ADDR_01, pos_up[X], pos_lo[X], 
		        cmd.acc_lo, cmd.acc_up, cmd.dec_lo, cmd.dec_up, 
		        cmd.op_speed_lo, cmd.op_speed_up, operating_mode, seq_pos, i + 1); // X
		    initSingleMotion(MODBUS_SLAVE_ADDR_02, pos_up[Y], pos_lo[Y], 
		        cmd.acc_lo, cmd.acc_up, cmd.dec_lo, cmd.dec_up,
		        cmd.op_speed_lo, cmd.op_speed_up, operating_mode, seq_pos, i + 1); // Y
            initSingleMotion(MODBUS_SLAVE_ADDR_03, pos_up[Z], pos_lo[Z], 
                cmd.acc_lo, cmd.acc_up, cmd.dec_lo, cmd.dec_up, 
                cmd.op_speed_lo, cmd.op_speed_up, operating_mode, seq_pos, i + 1); // Z	
             
            // This is important. You have to count the linked motions, because they need to be
            // subtracted from the motions to start. 4 linked motions only require 1 start.     
            if (operating_mode == MOTOR_OPM_LINK1 ||
                operating_mode == MOTOR_OPM_LINK2)
            {
                linked_motions++;
            }
	    }
    }
    
    if (linked_motions > 0)
    {
        // This is the recipe:
        // motions = motions - linked_motions
        // motions += (linked_motions / MOTOR_MAX_LINK_MOT);
        // if (linked_motions % MOTOR_MAX_LINK_MOT) > 0 then motions++;
        std::cout << "MOTIONS: " << motions << std::endl;
        motions = motions - linked_motions;
        motions = motions + (linked_motions / MOTOR_MAX_LINK_MOT);
        if ( (linked_motions % MOTOR_MAX_LINK_MOT) > 0) motions++;
        std::cout << "LINKED MOTIONS: " << linked_motions << std::endl;
        std::cout << "MOTIONS TO START AFTER CORRECTION: " << motions << std::endl;
    }
}

void StepperMotor::exit()
{
    std::cout << "StepperMotor::exit()" << std::endl;
    if (ctx != NULL)
    {
	    //modbus_close(ctx);
	    //modbus_free(ctx);    
    }
}

void StepperMotor::initSingleMotion(int slave, uint16_t pos_lo, uint16_t pos_up, 
    uint16_t acc_lo, uint16_t acc_up, uint16_t dec_lo, uint16_t dec_up, 
    uint16_t speed_lo, uint16_t speed_up, uint16_t operating_mode, uint16_t seq_pos,
    int off)
{
	uint16_t src[2];
	int n;
	// position is address 0x0402 and 0x0403 and specify position pos_up / pos_lo
	modbus_set_slave(ctx, slave);

	src[1] = pos_up;
	src[0] = pos_lo;
	n = modbus_write_registers(ctx, REG_MOTOR_POS + (off * 2), 2, src);
	//printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_PROC_TIME);
	
	// set operating speed
	src[1] = speed_up;
	src[0] = speed_lo;
	std::cout << "speed_up: " << speed_up << std::endl;
    std::cout << "speed_lo: " << speed_lo << std::endl;
	n = modbus_write_registers(ctx, REG_MOTOR_OP_SPEED + (off * 2), 2, src);
	//printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_PROC_TIME);
	
	// set acceleration
	src[1] = acc_up;
	src[0] = acc_lo;
	std::cout << "acc_up: " << acc_up << std::endl;
    std::cout << "acc_lo: " << acc_lo << std::endl;
	n = modbus_write_registers(ctx, REG_MOTOR_ACC + (off * 2), 2, src);
	//printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_PROC_TIME);	
	
	// set deceleration
	src[1] = dec_up;
	src[0] = dec_lo;
	std::cout << "dec_up: " << dec_up << std::endl;
    std::cout << "dec_lo: " << dec_lo << std::endl;	
	n = modbus_write_registers(ctx, REG_MOTOR_DEC + (off * 2), 2, src);
	//printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_PROC_TIME);		

	// set to absolute positioning
	n = modbus_write_register(ctx, REG_MOTOR_MPOS + (off * 1), 0x01);	
	//printf("errno: %s\n", modbus_strerror(errno));
	usleep(MODBUS_MAX_PROC_TIME);
	
	// set operating mode
	n = modbus_write_register(ctx, REG_MOTOR_OPM + (off * 1), operating_mode);
	usleep(MODBUS_MAX_PROC_TIME);
	
	// set to sequential positioning
	n = modbus_write_register(ctx, REG_MOTOR_SQPS + (off * 1), seq_pos);
	usleep(MODBUS_MAX_PROC_TIME);

    /*
	if (slave == 1)
	{
		src[0] = 0;
		src[1] = 0;
		n = modbus_read_registers(ctx, 0x0019, 1, src);
		usleep(MODBUS_MAX_PROC_TIME);
		printf("ACCELERATION RATE FOR SELECT DATA NUMBER (LOWER): %x\n", src[0]);
	}
	*/
}

uint16_t StepperMotor::angleToStep(double x, bool& invalid_motion)
{
	if (x != x)
    	{
	    invalid_motion = true;
 	}
	else
	{
		invalid_motion = false;
	}
	return static_cast<uint16_t>(round((90.0 + x) / MOTOR_STEP_ANGLE));
}

void StepperMotor::test()
{
}
