/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 */

#include <iostream>
#include <ros/ros.h>
#include <par_kinematics/coord.h>
#include <par_kinematics/deltarobot.h>
#include <par_kinematics/par_kinematics.h>
#include <par_trajectory_planning/config.h>

static const int QUEUE_SIZE = 1000;
static DeltaRobot* deltaRobot;

/**
 * @brief This is the callback function responsible for publishing angles in radians.
 * @param coord object representing XYZ coordinates. 
 */
bool IK_solver(par_kinematics::coord::Request& req,
               par_kinematics::coord::Response& res)
{
    std::cout << "Inverse kinematics request received." << std::endl;
    // y and z are changed. this is how the model works. you could of course fix this
    // by rewriting the code, but at the moment i don't have the time to do so.
	double x = req.x;
	double y = req.z;
	double z = req.y;

    Point goal(x, y, z);
	deltaRobot->moveto(goal);    
	
	res.angles[X] = Util::deg(deltaRobot->servo[X]);
	res.angles[Y] = Util::deg(deltaRobot->servo[Y]);
	res.angles[Z] = Util::deg(deltaRobot->servo[Z]); 	
	
	std::cout << "X: " << Util::deg(deltaRobot->servo[X]) << std::endl;
	std::cout << "Y: " << Util::deg(deltaRobot->servo[Y]) << std::endl;
	std::cout << "Z: " << Util::deg(deltaRobot->servo[Z]) << std::endl;

    return true;    
}

int main(int argc, char **argv)
{
    std::cout << "par_kinematics_subscriber." << std::endl;
    ros::init(argc, argv, "par_kinematics_subscriber");
    ros::NodeHandle n;
    deltaRobot = new DeltaRobot(DELTA_ROBOT_HIP_LEN, DELTA_ROBOT_ANK_LEN, 
                        DELTA_ROBOT_BS_SZ, DELTA_ROBOT_EF_SZ);
    
    ros::ServiceServer service = n.advertiseService("coord", IK_solver);
    
    ros::spin();
    
    return 0;
}
