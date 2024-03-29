/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <par_ui/par_config.h>
#include <par_ui/par_utils.h>
#include <par_kinematics/coord.h>
#include <par_trajectory_planning/commands.h>
#include <par_trajectory_planning/config.h>
#include <tinyxml/tinyxml.h>

static const int QUEUE_SIZE = 1000;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "par_ui");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    ros::ServiceClient coord_client = n.serviceClient<par_kinematics::coord>("coord");
    ros::Publisher chatter_pub_cmd = n.advertise<par_trajectory_planning::commands>("commands", QUEUE_SIZE);
    	    
    par_trajectory_planning::commands cmd;
    par_kinematics::coord coords;    	    
    Config config(coord_client, coords, cmd, chatter_pub_cmd);
    std::string file;
    int x = 1;
    while(ros::ok() && x)
    {
        x = menu();
        switch(x)
        {
            case MENU_INIT_COMM:
                cmd.option = MENU_INIT_COMM;
                chatter_pub_cmd.publish(cmd);
            break;
            case MENU_INIT_MOTOR: 
                cmd.option = MENU_INIT_MOTOR;
                chatter_pub_cmd.publish(cmd);
            break;
            case MENU_CONF_SIN_MOT:
                cmd.option = MENU_CONF_SIN_MOT;
		        configure_single_motion(cmd);
		        chatter_pub_cmd.publish(cmd);
            break;
            case MENU_CONF_PTP_MOT:
                cmd.option = MENU_CONF_PTP_MOT;
                configure_PTP_motion(coord_client, coords, cmd);
                chatter_pub_cmd.publish(cmd);
            break;
	        case MENU_RD_CONF_FILE:
                cmd.option = MENU_RD_CONF_FILE;
                pick_configuration_file("/etc/ROS", file);
		        config.read(file);
	            break;
            case MENU_START_MOT:
                cmd.option = MENU_START_MOT;
                chatter_pub_cmd.publish(cmd);
            break;
	        case MENU_STOP_MOT:
		        cmd.option = MENU_STOP_MOT;
		        chatter_pub_cmd.publish(cmd);
	        break;
	        case MENU_START_HM:
	            cmd.option = MENU_START_HM;
	            configure_homing(cmd);
	            chatter_pub_cmd.publish(cmd);
	        break;
	        case MENU_START_TEST:
		        cmd.option = MENU_START_TEST;
		        chatter_pub_cmd.publish(cmd);
		    break;
            case MENU_EXIT:
                cmd.option = MENU_EXIT;
                chatter_pub_cmd.publish(cmd);
            break;
        }
    }    
    
    return 0;
}
