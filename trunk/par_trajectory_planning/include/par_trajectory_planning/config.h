#ifndef _CONFIG_H_
#define _CONFIG_H_


const double DELTA_ROBOT_HIP_LEN = 62.0;  // hip length
const double DELTA_ROBOT_ANK_LEN = 150.0; // ankle length
const double DELTA_ROBOT_BS_SZ   = 101.4; // base size
const double DELTA_ROBOT_EF_SZ   = 46.19; // effector size

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

const int MODBUS_MAX_PROC_TIME  = 4000; // in usec
const int MODBUS_MAX_BCAST_TIME = 7500; // in usec
const int MODBUS_TIMEOUT_BEGIN  = 50000; // in usec
const int MODBUS_TIMEOUT_END    = 50000;
const int MODBUS_SLAVE_ADDR_01  = 0x01;
const int MODBUS_SLAVE_ADDR_02  = 0x02;
const int MODBUS_SLAVE_ADDR_03  = 0x03;

const uint16_t MOTOR_SPEED_UP   = 0xA120; // working default 0x2800
const uint16_t MOTOR_SPEED_LO   = 0x0007;
const uint16_t MOTOR_ACC_UP     = 0x1388;
const uint16_t MOTOR_ACC_LO     = 0x0000;
const uint16_t MOTOR_DEC_UP     = 0x1388;
const uint16_t MOTOR_DEC_LO     = 0x0000;
const double   MOTOR_STEP_ANGLE = 0.072;

const uint16_t REG_MOTOR_POS   = 0x0400; // 2 bytes per offset 
const uint16_t REG_MOTOR_SPEED = 0x0500; // 2 bytes per offset
const uint16_t REG_MOTOR_ABS   = 0x0600; // 1 byte per offset
const uint16_t REG_MOTOR_OPM   = 0x0700; // 1 byte per offset
const uint16_t REG_MOTOR_SQPS  = 0x0800; // 1 byte per offset 
const uint16_t REG_MOTOR_ACC   = 0x0900; // 2 bytes per offset
const uint16_t REG_MOTOR_DEC   = 0x0A00; // 2 bytes per offset
const uint16_t REG_MOTOR_DT    = 0x0C01; // 1 byte per offset


const int XYZ_DIV_FACTOR = 16;

const int X = 0;
const int Y = 1;
const int Z = 2;

#endif
