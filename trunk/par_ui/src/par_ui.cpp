#include <iostream>
#include <ros/ros.h>
#include <par_kinematics/coord.h>
#include <par_trajectory_planning/commands.h>
#include <par_trajectory_planning/config.h>

static const int QUEUE_SIZE = 1000;

void configure_single_motion(par_trajectory_planning::commands& cmd)
{
	cmd.abs_pos.clear();
	uint16_t pos_up[3];
	uint16_t pos_lo[3];   
	int motions = 0;
	
	std::cout << "Enter amount of single motions to configure: " << std::endl; 
	std::cin >> motions;
	
	int i = 0;
	while(i < motions)
	{
	    pos_up[0] = 0;
	    std::cout << "pos lo for axis 1: ";
	    std::cin >> pos_lo[0];
	    if (pos_lo[0] & 0x8000) pos_up[0] = 0xFFFF;

	    pos_up[1] = 0;
	    std::cout << "pos lo for axis 2: ";
	    std::cin >> pos_lo[1];
	    if (pos_lo[1] & 0x8000) pos_up[1] = 0xFFFF;

	    pos_up[2] = 0;
	    std::cout << "pos lo for axis 3: ";
	    std::cin >> pos_lo[2];
	    if (pos_lo[2] & 0x8000) pos_up[2] = 0xFFFF;

	    cmd.abs_pos.push_back(pos_up[X]);
	    cmd.abs_pos.push_back(pos_lo[X]);

	    cmd.abs_pos.push_back(pos_up[Y]);
	    cmd.abs_pos.push_back(pos_lo[Y]);

	    cmd.abs_pos.push_back(pos_up[Z]);
	    cmd.abs_pos.push_back(pos_lo[Z]);	    
	    
	    i++;
	}
}

void configure_PTP_motion(ros::ServiceClient& coord_client, par_kinematics::coord& coords,
			  par_trajectory_planning::commands& cmd)
{   
    cmd.xyz_pos.clear();
    int motions = 0;
	std::cout << "Enter amount of PTP motions to configure: " << std::endl; 
	std::cin >> motions;
	
	int i = 0;
	while(i < motions)    
    {
        std::cout << "X: ";
        std::cin >> coords.request.x;
        std::cout << "Y: ";
        std::cin >> coords.request.y;
        std::cout << "Z: ";
        std::cin >> coords.request.z;
        
        if (coord_client.call(coords))
        {
            std::cout << "success." << std::endl;
        }
        else
        {
            std::cout << "failure." << std::endl;
        }
        
        cmd.xyz_pos.push_back(coords.response.angles[X]);
        cmd.xyz_pos.push_back(coords.response.angles[Y]);
        cmd.xyz_pos.push_back(coords.response.angles[Z]);

        std::cout << "coords.request.x: " << coords.request.x << std::endl;
        std::cout << "coords.request.y: " << coords.request.y << std::endl;
        std::cout << "coords.request.z: " << coords.request.z << std::endl;
        
        i++;
    }
}

int menu()
{
    std::cout << "[" << MENU_INIT_COMM      << "] init communication"       << std::endl;
    std::cout << "[" << MENU_INIT_MOTOR     << "] init motors"              << std::endl;
    std::cout << "[" << MENU_CONF_SIN_MOT   << "] configure single motion"  << std::endl;
    std::cout << "[" << MENU_CONF_PTP_MOT   << "] configure PTP motion"     << std::endl;
    std::cout << "[" << MENU_START_MOT      << "] start motion"             << std::endl;
    std::cout << "[" << MENU_EXIT           << "] exit"                     << std::endl;
    
    int x = 0;
    std::cin >> x;
    return x;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "par_ui");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    ros::ServiceClient coord_client = n.serviceClient<par_kinematics::coord>("coord");
    ros::Publisher chatter_pub_cmd = n.advertise<par_trajectory_planning::commands>("commands", QUEUE_SIZE);
    	    
    
    par_trajectory_planning::commands cmd;
    par_kinematics::coord coords;
    int x = 1;
    while(ros::ok() && x)
    {
        x = menu();
        switch(x)
        {
            case MENU_INIT_COMM:
                cmd.option = MENU_INIT_COMM;
            break;
            case MENU_INIT_MOTOR: 
                cmd.option = MENU_INIT_MOTOR;
            break;
            case MENU_CONF_SIN_MOT:
                cmd.option = MENU_CONF_SIN_MOT;
		        configure_single_motion(cmd);
            break;
            case MENU_CONF_PTP_MOT:
                cmd.option = MENU_CONF_PTP_MOT;
                configure_PTP_motion(coord_client, coords, cmd);
            break;
            case MENU_START_MOT:
                cmd.option = MENU_START_MOT;
            break;
            case MENU_EXIT:
                cmd.option = MENU_EXIT;
                std::cout << "Bye." << std::endl;
            break;
        }
        chatter_pub_cmd.publish(cmd);
    }    
    
    return 0;
}
