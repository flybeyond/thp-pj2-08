#include <iostream>
#include <ros/ros.h>
#include <par_kinematics/coord.h>

static const int QUEUE_SIZE = 1000;

int main(int argc, char **argv)
{
    std::cout << "par_ui" << std::endl;
    ros::init(argc, argv, "par_ui");
    ros::NodeHandle n;
    
    ros::Publisher chatter_pub = n.advertise<par_kinematics::coord>("coord", QUEUE_SIZE);
    
    ros::Rate loop_rate(1);
    
    int count = 0;
    par_kinematics::coord coord;
    while (ros::ok())
    {
        std::cout << "Enter X: "; std::cin >> coord.XYZ[0];
        std::cout << "Enter Y: "; std::cin >> coord.XYZ[1];
        std::cout << "Enter Z: "; std::cin >> coord.XYZ[2];
        
        chatter_pub.publish(coord);
        //ros::spinOnce();
        //loop_rate.sleep();
        ++count;
    }   
    
    return 0;
}
