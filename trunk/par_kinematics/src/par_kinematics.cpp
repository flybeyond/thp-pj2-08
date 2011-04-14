#include <iostream>
#include <ros/ros.h>
#include <par_kinematics/coord.h>
#include <par_kinematics/kinematics.h>
#include <par_trajectory_planning/angles.h>

static const int QUEUE_SIZE = 1000;
static Kinematics kinematic_solver;
static ros::Publisher chatter_pub;

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
void IK_solver(const par_kinematics::coord& coord)
{
    std::cout << "[seq: " << coord.header.seq << "]" << std::endl;
    std::cout << "[" << coord.XYZ[0] << ", " << coord.XYZ[1] << ", " << coord.XYZ[2] << "]" << std::endl;
    
    if ( kinematic_solver.getPositionIK(coord.XYZ[0], coord.XYZ[1], coord.XYZ[2]) )
    {
        std::cout << "[kinematics] existing point found." << std::endl;
        std::cout << "[kinematics] theta1: " << kinematic_solver.getTheta1() << std::endl;
        std::cout << "[kinematics] theta2: " << kinematic_solver.getTheta2() << std::endl;
        std::cout << "[kinematics] theta3: " << kinematic_solver.getTheta3() << std::endl;
    }
    else
    {
        std::cout << "[kinematics] non-existing point." << std::endl;
    }
    /* 
     * For debugging purposes every message gets published; valid or invalid points.
     */
    par_trajectory_planning::angles angles;
    angles.XYZ[0] = rad(coord.XYZ[0]);
    angles.XYZ[1] = rad(coord.XYZ[1]);
    angles.XYZ[2] = rad(coord.XYZ[2]);
    chatter_pub.publish(angles);
}

int main(int argc, char **argv)
{
    std::cout << "par_kinematics_subscriber." << std::endl;
    ros::init(argc, argv, "par_kinematics_subscriber");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("coord", QUEUE_SIZE, IK_solver);
    chatter_pub = n.advertise<par_trajectory_planning::angles>("angles", QUEUE_SIZE);
    
    ros::spin();
    
    return 0;
}
