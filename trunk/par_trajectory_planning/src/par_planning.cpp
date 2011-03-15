#include <iostream>
#include <ros/ros.h>
#include <par_trajectory_planning/angles.h>

static const int QUEUE_SIZE = 1000;

/**
 * @brief This is the callback function responsible for receiving angles in radian.
 * @param angles object representing XYZ coordinates in radians.
 */
void MO_angles(const par_trajectory_planning::angles& angles)
{
    // communicate angles to motion interface.
}

int main(int argc, char **argv)
{
    std::cout << "par_trajectory_planning." << std::endl;
    ros::init(argc, argv, "par_trajectory_planning");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("angles", QUEUE_SIZE, MO_angles);
    
    ros::spin();
    
    return 0;
}
