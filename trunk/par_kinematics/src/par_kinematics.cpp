#include <iostream>
#include <ros/ros.h>
#include <par_kinematics/coord.h>
#include <par_kinematics/kinematics.h>
#include <par_trajectory_planning/angles.h>
#include "/home/wouter/ros_packages/thp-pj2-08/par_trajectory_planning/include/par_trajectory_planning/config.h"

static const int QUEUE_SIZE = 1000;
static Kinematics kinematic_solver;

/**
 * @brief This function converts degrees to radians.
 * @param deg theta in degrees
 */
double rad(double deg) 
{
    return double(deg/180 * pi);
}

/**
 * @brief This is the callback function responsible for publishing angles in radians.
 * @param coord object representing XYZ coordinates. 
 */
bool IK_solver(par_kinematics::coord::Request& req,
               par_kinematics::coord::Response& res)
{
    std::cout << "Inverse kinematics request received." << std::endl;
    
    if ( kinematic_solver.getPositionIK(req.x, req.y, req.z) )
    {
        std::cout << "[kinematics] existing point found." << std::endl;
        std::cout << "[kinematics] theta1: " << kinematic_solver.getTheta1() << std::endl;
        std::cout << "[kinematics] theta2: " << kinematic_solver.getTheta2() << std::endl;
        std::cout << "[kinematics] theta3: " << kinematic_solver.getTheta3() << std::endl;
        
        res.angles[X] = rad(kinematic_solver.getTheta1());
        res.angles[Y] = rad(kinematic_solver.getTheta2());
        res.angles[Z] = rad(kinematic_solver.getTheta3());
    }
    else
    {
        std::cout << "[kinematics] non-existing point." << std::endl;
        res.angles[X] = 0;
        res.angles[Y] = 0;
        res.angles[Z] = 0;
    }

    return true;    
}

int main(int argc, char **argv)
{
    std::cout << "par_kinematics_subscriber." << std::endl;
    ros::init(argc, argv, "par_kinematics_subscriber");
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("coord", IK_solver);
    
    ros::spin();
    
    return 0;
}
