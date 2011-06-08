/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

/**
 * Various constants.
 */
const int XYZ_DIV_FACTOR = 16;
const int X = 0;
const int Y = 1;
const int Z = 2;

/**
 * Menu constants used by the interface. Declared in this header, because
 * this constants are used to start the appropriate action.
 */
const int MENU_INIT_COMM    = 1;
const int MENU_INIT_MOTOR   = 2;
const int MENU_CONF_SIN_MOT = 3;
const int MENU_CONF_PTP_MOT = 4;
const int MENU_RD_CONF_FILE = 5;
const int MENU_START_MOT    = 6;
const int MENU_STOP_MOT	    = 7;
const int MENU_START_HM	    = 8;
const int MENU_START_TEST   = 9;
const int MENU_EXIT         = 0;

/**
 * Modbus constants.
 */
const int MODBUS_MAX_PROC_TIME  = 4000;  // in usec
const int MODBUS_MAX_BCAST_TIME = 7500;  // in usec
const int MODBUS_TIMEOUT_BEGIN  = 50000; // in usec
const int MODBUS_TIMEOUT_END    = 50000;
const int MODBUS_SLAVE_ADDR_01  = 0x01;
const int MODBUS_SLAVE_ADDR_02  = 0x02;
const int MODBUS_SLAVE_ADDR_03  = 0x03;

/**
 * Constants for configuring the controllers/motors.
 */
const uint16_t MOTOR_OP_SPEED_UP = 0xA120; 
const uint16_t MOTOR_OP_SPEED_LO = 0x0007;
const uint16_t MOTOR_ST_SPEED_UP = 0x03E8;
const uint16_t MOTOR_ST_SPEED_LO = 0x0000;
const uint16_t MOTOR_ACC_UP      = 0x1388;
const uint16_t MOTOR_ACC_LO      = 0x0000;
const uint16_t MOTOR_DEC_UP      = 0x1388;
const uint16_t MOTOR_DEC_LO      = 0x0000;
const uint16_t MOTOR_OPM_SINGLE  = 0x0000;
const uint16_t MOTOR_OPM_LINK1   = 0x0001;
const uint16_t MOTOR_OPM_LINK2   = 0x0002;
const double   MOTOR_STEP_ANGLE  = 0.072;

/**
 * Register address constants. See datasheet for more registers.
 */
const uint16_t REG_MOTOR_ST_SPEED = 0x0228; // 0 bytes offset, starting speed motor.
const uint16_t REG_MOTOR_POS      = 0x0400; // 2 bytes per offset, motor position.
const uint16_t REG_MOTOR_OP_SPEED = 0x0500; // 2 bytes per offset, motor operating speed.
const uint16_t REG_MOTOR_MPOS     = 0x0600; // 1 byte per offset, positioning mode.
const uint16_t REG_MOTOR_OPM      = 0x0700; // 1 byte per offset, motor operating mode.
const uint16_t REG_MOTOR_SQPS     = 0x0800; // 1 byte per offset, sequential positioning.
const uint16_t REG_MOTOR_ACC      = 0x0900; // 2 bytes per offset, motor acceleration.
const uint16_t REG_MOTOR_DEC      = 0x0A00; // 2 bytes per offset, motor deceleration.
const uint16_t REG_MOTOR_DT       = 0x0C01; // 1 byte per offset, dwell time used for linked motions.

#endif
