/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 */
 
#include <iostream>
#include <ros/ros.h>
#include <par_trajectory_planning/commands.h>
#include <par_trajectory_planning/steppermotor.h>
#include <par_trajectory_planning/config.h>
#include <cerrno>

static const int QUEUE_SIZE = 1000;
static StepperMotor* stepperMotor;

/**
 * @brief This is the callback function responsible for receiving and processing commands from the user.
 * @param cmd Object with controller instructions.
 */
void commands(const par_trajectory_planning::commands& cmd)
{
    switch(cmd.option)
    {
        case MENU_INIT_COMM:
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
	    case MENU_START_HM:
            stepperMotor->confSingleMotion(cmd);
	        stepperMotor->start();
	    break;
	    case MENU_STOP_MOT:
	        stepperMotor->stop();
	    break;
	    case MENU_START_TEST:
	        stepperMotor->test();
	    break;
        case MENU_EXIT:
            stepperMotor->exit();
        break;  
    }
}

int main(int argc, char **argv)
{
    std::cout << "par_trajectory_planning." << std::endl;
    ros::init(argc, argv, "par_trajectory_planning", ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
    ros::NodeHandle n;
    /**
     * Important note!
     * You cannot move the initialization of the modbus interface inside the class' constructor.
     * I don't know the exact reason for this, but GDB shows some memory leakage and
     * initialization of the object fails. 
     */
    modbus_t* ctx = modbus_new_rtu("/dev/ttyS0", 115200, 'N', 8, 1); 
    stepperMotor = new StepperMotor(ctx);
	ros::Subscriber sub = n.subscribe("commands", QUEUE_SIZE, commands);
    ros::spin();
    
    return 0;
}
