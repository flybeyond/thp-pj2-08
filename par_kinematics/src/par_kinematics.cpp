#include <iostream>
#include <ros/ros.h>
#include <par_kinematics/coord.h>
#include <par_kinematics/kinematics.h>


static const int QUEUE_SIZE = 1000;
static Kinematics kinematic_solver;

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
}

int main(int argc, char **argv)
{
    std::cout << "par_kinematics_subscriber." << std::endl;
    ros::init(argc, argv, "par_kinematics_subscriber");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("coord", QUEUE_SIZE, IK_solver);
    
    ros::spin();
    
    return 0;
}
