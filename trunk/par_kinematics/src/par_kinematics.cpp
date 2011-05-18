#include <iostream>
#include <ros/ros.h>
#include <par_kinematics/coord.h>
#include <par_kinematics/deltarobot.h>
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
    
	double x = req.x;
	double y = req.y;
	double z = req.z;
	// y and z are reversed, yes, this is nasty.
	// for now, it's a quick fix that works as expected. 
    Point goal(x, z, y);
	deltaRobot->moveto(goal);    
	
    std::cout << "deltaRobot->servo[X]: " << deltaRobot->servo[X] << std::endl;
	std::cout << "deltaRobot->servo[Y]: " << deltaRobot->servo[Y] << std::endl;
	std::cout << "deltaRobot->servo[Z]: " << deltaRobot->servo[Z] << std::endl;
	
	std::cout << "deltaRobot->servo[X] deg: " << Util::deg(deltaRobot->servo[X]) << std::endl;
	std::cout << "deltaRobot->servo[Y] deg: " << Util::deg(deltaRobot->servo[Y]) << std::endl;
	std::cout << "deltaRobot->servo[Z] deg: " << Util::deg(deltaRobot->servo[Z]) << std::endl;
	res.angles[X] = Util::deg(deltaRobot->servo[X]);
	res.angles[Y] = Util::deg(deltaRobot->servo[Y]);
	res.angles[Z] = Util::deg(deltaRobot->servo[Z]); 
    

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
