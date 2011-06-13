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
    motions = 0;

    // It is important to clear the operation data register with default settings.
    // E.g., if more than 4 motions are linked the system won't work.
    modbus_set_slave(ctx, MODBUS_BCAST_ADDR);
    int i;
    for(i=0; i<63; i++)
    {
	    // set operating mode
	    int n = modbus_write_register(ctx, REG_MOTOR_OPM + (i + 1), MOTOR_OPM_SINGLE);
	    usleep(MODBUS_MAX_PROC_TIME);	

        // disable sequential positioning
	    n = modbus_write_register(ctx, REG_MOTOR_SQPS + (i + 1), 0x00);
	    usleep(MODBUS_MAX_PROC_TIME);        

        // set positions
	    uint16_t src[2] = {0x0000, 0x0000};
	    n = modbus_write_registers(ctx, REG_MOTOR_POS + (i + 1) * 2, 2, src);
    	usleep(MODBUS_MAX_PROC_TIME);

	    // set to absolute positioning
	    n = modbus_write_register(ctx, REG_MOTOR_MPOS + (i + 1), 0x01);	
	    usleep(MODBUS_MAX_PROC_TIME);
    }
    std::cout << "init successful" << std::endl;
}

void StepperMotor::start()
{
    uint16_t stat_ax01[1], stat_ax02[1], stat_ax03[1];
    int i, n, j;
    
    j = 0;
    std::cout << "REPEAT MOTIONS: " << repeat_motions << std::endl;
    std::cout << "MOTIONS: " << motions << std::endl;

    while(j < repeat_motions)
    {
        for(i=0; i<motions-1; i++)
        {
	        modbus_set_slave(ctx, 0);
            n = modbus_write_register(ctx, 0x001E, 0x2000);
	        usleep(MODBUS_MAX_BCAST_TIME);
	        // turn start input on
	        n = modbus_write_register(ctx, 0x001E, 0x2100 );
	        usleep(MODBUS_MAX_BCAST_TIME);
	        // turn start input off
	        n = modbus_write_register(ctx, 0x001E, 0x2000 ); 
	        usleep(MODBUS_MAX_BCAST_TIME);

	        stat_ax01[0] = stat_ax02[0] = stat_ax03[0] = 0x0000;
            /**
             * This loop checks if the motor is ready to receive new commands.
             * See the config file for the register address en for details
             * the datasheet.
             */
	        while( (stat_ax01[0] != MOTOR_READY) ||
	               (stat_ax02[0] != MOTOR_READY) ||
	               (stat_ax03[0] != MOTOR_READY) )
	        {
		        modbus_set_slave(ctx, MODBUS_SLAVE_ADDR_01);
		        n = modbus_read_registers(ctx, 0x0020, 1, stat_ax01); 
		        usleep(MODBUS_MAX_PROC_TIME);
                
		        modbus_set_slave(ctx, MODBUS_SLAVE_ADDR_02);
		        n = modbus_read_registers(ctx, 0x0020, 1, stat_ax02); 
		        usleep(MODBUS_MAX_PROC_TIME);

		        modbus_set_slave(ctx, MODBUS_SLAVE_ADDR_03);
		        n = modbus_read_registers(ctx, 0x0020, 1, stat_ax03); 
		        usleep(MODBUS_MAX_PROC_TIME);
	        }
	        std::cout << "motor can receive new command\n" << std::endl;
        }  
        j++;
    }
    stop_motion = false;
    // This is important. Sequential positioning is used. So if a motion is stopped you have to
    // manually set the operation number to zero again. Otherwise, the register's position stays
    // at the last executed position + 1.
    modbus_set_slave(ctx, MODBUS_BCAST_ADDR);
	n = modbus_write_register(ctx, 0x001E, 0x0000 /* + i */ );
    usleep(MODBUS_MAX_PROC_TIME);
}

void StepperMotor::stop()
{
    /**
     * start() is blocking; implement threads to set this variable to false to stop
     * a motion in progress.
     */
}

void StepperMotor::initCom()
{	
    // Enable this if you want to have modbus debugging information.
	// modbus_set_debug(ctx, 1);
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
            MOTOR_ACC_LO, MOTOR_ACC_UP, MOTOR_DEC_LO, MOTOR_DEC_UP, 
            MOTOR_OP_SPEED_LO, MOTOR_OP_SPEED_UP, MOTOR_OPM_SINGLE, 0x00, i + 1); // X

        initSingleMotion(MODBUS_SLAVE_ADDR_02, cmd.abs_pos[ 2 + (i * 6) ], cmd.abs_pos[ 3 + (i * 6) ],
            MOTOR_ACC_LO, MOTOR_ACC_UP, MOTOR_DEC_LO, MOTOR_DEC_UP, 
            MOTOR_OP_SPEED_LO, MOTOR_OP_SPEED_UP, MOTOR_OPM_SINGLE, 0x00, i + 1); // Y

        initSingleMotion(MODBUS_SLAVE_ADDR_03, cmd.abs_pos[ 4 + (i * 6) ], cmd.abs_pos[ 5 + (i * 6) ], 
            MOTOR_ACC_LO, MOTOR_ACC_UP, MOTOR_DEC_LO, MOTOR_DEC_UP,
            MOTOR_OP_SPEED_LO, MOTOR_OP_SPEED_UP, MOTOR_OPM_SINGLE, 0x00, i + 1); // Z
    }

}

void StepperMotor::confPTPMotion(const par_trajectory_planning::commands& cmd)
{
    init();
    motions = cmd.xyz_pos.size() / 3;
    repeat_motions = cmd.repeat_motions;
    if (repeat_motions == 0) repeat_motions = 1;    

    uint16_t src[2];
    modbus_set_slave(ctx, MODBUS_BCAST_ADDR);
	src[1] = cmd.st_speed_lo;
	src[0] = cmd.st_speed_up;
	modbus_write_registers(ctx, REG_MOTOR_ST_SPEED, 2, src);
	usleep(MODBUS_MAX_PROC_TIME);    

    int i;
    int linked_motions = 0;
    int single_motions = 0;
    uint16_t seq_pos   = 0x01; // enable sequential positioning by default.
    uint16_t pos_lo[3];
    uint16_t pos_up[3];
    bool invalid_motion[3] = {false, false, false};
    for(i=0; i<motions; i++)  
    {        
        // unsigned integer do not support negative numbers. Unsigned data is transferred
        // to the device. The device supports two complements numbers so make it negative
        // yourself.
        pos_up[X] = angleToStep( cmd.xyz_pos[ 0 + (i * 3) ], invalid_motion[X] );
        pos_lo[X] = ( pos_up[X] & 0x8000 ) ? 0xFFFF : 0x00;
        
        pos_up[Y] = angleToStep( cmd.xyz_pos[ 1 + (i * 3) ], invalid_motion[Y] );
        pos_lo[Y] = ( pos_up[Y] & 0x8000 ) ? 0xFFFF : 0x00;
        
        pos_up[Z] = angleToStep( cmd.xyz_pos[ 2 + (i * 3) ], invalid_motion[Z] );
        pos_lo[Z] = ( pos_up[Z] & 0x8000 ) ? 0xFFFF : 0x00;

        uint16_t operating_mode = cmd.operating_mode[i];

	    if (invalid_motion[X] ||
            invalid_motion[Y] ||
            invalid_motion[Z] )
	    {
		    std::cout << "INVALID MOTION; NOT CONFIGURED!" << std::endl;
	    }
	    else
	    {
		    initSingleMotion(MODBUS_SLAVE_ADDR_01, pos_lo[X], pos_up[X], 
		        cmd.acc_lo, cmd.acc_up, cmd.dec_lo, cmd.dec_up, 
		        cmd.op_speed_lo, cmd.op_speed_up, operating_mode, seq_pos, i + 1); // X
		    initSingleMotion(MODBUS_SLAVE_ADDR_02, pos_lo[Y], pos_up[Y], 
		        cmd.acc_lo, cmd.acc_up, cmd.dec_lo, cmd.dec_up,
		        cmd.op_speed_lo, cmd.op_speed_up, operating_mode, seq_pos, i + 1); // Y
            initSingleMotion(MODBUS_SLAVE_ADDR_03, pos_lo[Z], pos_up[Z], 
                cmd.acc_lo, cmd.acc_up, cmd.dec_lo, cmd.dec_up, 
                cmd.op_speed_lo, cmd.op_speed_up, operating_mode, seq_pos, i + 1); // Z	
             
            // This is important. You have to count the single motions, because that are the
            // only ones you have to manually start.
            if (operating_mode == MOTOR_OPM_SINGLE)
            {
                single_motions++;
            }
            else
            {
                linked_motions++;
            }
	    }
    }

    // safe guard check: if the first entry is a linked motion
    // a start for that motion must be done.
    if (cmd.operating_mode[0] == MOTOR_OPM_LINK1 ||
        cmd.operating_mode[0] == MOTOR_OPM_LINK2)
    {
        if ( (motions % MOTOR_MAX_LINK_MOT == 0) )
        {
            motions = single_motions;
        }
        else
        {
            motions = ++single_motions;
        }
    }
    // This has to be done to overcome a loop limit issue. If it has none linked motions
    // a motion is missed.
    if (linked_motions == 0)
    { 
        motions++;
    }

    std::cout << "MOTIONS TO START: " << motions << std::endl;
}

void StepperMotor::exit()
{
    /**
     * Probably should do some cleaning up right here...
     */
}

void StepperMotor::initSingleMotion(int slave, uint16_t pos_lo, uint16_t pos_up, 
    uint16_t acc_lo, uint16_t acc_up, uint16_t dec_lo, uint16_t dec_up, 
    uint16_t speed_lo, uint16_t speed_up, uint16_t operating_mode, uint16_t seq_pos,
    int off)
{
	uint16_t src[2];
	int n;
	modbus_set_slave(ctx, slave);

	src[1] = pos_up;
	src[0] = pos_lo;
	std::cout << "src[1]: " << src[1] << std::endl;
	std::cout << "src[0]: " << src[0] << std::endl;
	n = modbus_write_registers(ctx, REG_MOTOR_POS + (off * 2), 2, src);
	usleep(MODBUS_MAX_PROC_TIME);
	
	// set operating speed
	src[1] = speed_up;
	src[0] = speed_lo;
	n = modbus_write_registers(ctx, REG_MOTOR_OP_SPEED + (off * 2), 2, src);
	usleep(MODBUS_MAX_PROC_TIME);
	
	// set acceleration
	src[1] = acc_up;
	src[0] = acc_lo;
	n = modbus_write_registers(ctx, REG_MOTOR_ACC + (off * 2), 2, src);
	usleep(MODBUS_MAX_PROC_TIME);	
	
	// set deceleration
	src[1] = dec_up;
	src[0] = dec_lo;
	n = modbus_write_registers(ctx, REG_MOTOR_DEC + (off * 2), 2, src);
	usleep(MODBUS_MAX_PROC_TIME);		

	// set to absolute positioning
	n = modbus_write_register(ctx, REG_MOTOR_MPOS + (off * 1), 0x01);	
	usleep(MODBUS_MAX_PROC_TIME);
	
	// set operating mode
	n = modbus_write_register(ctx, REG_MOTOR_OPM + (off * 1), operating_mode);
	usleep(MODBUS_MAX_PROC_TIME);
	
	// set to sequential positioning
	n = modbus_write_register(ctx, REG_MOTOR_SQPS + (off * 1), seq_pos);
	usleep(MODBUS_MAX_PROC_TIME);
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
    /**
     * This method has no use at the moment. I used it when developing for testing purposes; hence the name ;)
     * It's integrated in the menu so it's an easy to use way for e.g. testing control parameters.
     */
}
