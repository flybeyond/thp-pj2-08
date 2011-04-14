#include <iostream>
#include <ros/ros.h>
#include <par_trajectory_planning/angles.h>
#include <par_trajectory_planning/steppermotor.h>

static const int QUEUE_SIZE = 1000;
static StepperMotor* stepperMotor;

/**
 * @brief This is the callback function responsible for receiving angles in radian.
 * @param angles object representing XYZ coordinates in radians.
 */
void MO_angles(const par_trajectory_planning::angles& angles)
{
    // communicate angles to motion interface.
    std::cout << "[trajectory planning] theta1: " << angles.XYZ[0] << std::endl;
    std::cout << "[trajectory planning] theta2: " << angles.XYZ[1] << std::endl;
    std::cout << "[trajectory planning] theta3: " << angles.XYZ[2] << std::endl;
    stepperMotor->startPTPMotion(angles.XYZ[0], angles.XYZ[1], angles.XYZ[2]);
}

int main(int argc, char **argv)
{
    std::cout << "par_trajectory_planning." << std::endl;
    ros::init(argc, argv, "par_trajectory_planning");
    ros::NodeHandle n;
    stepperMotor = new StepperMotor();
    stepperMotor->init();

    ros::Subscriber sub = n.subscribe("angles", QUEUE_SIZE, MO_angles);

    ros::spin();

    return 0;
}
