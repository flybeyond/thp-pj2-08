/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 */

#include <par_ui/par_utils.h>

void push_angles(ros::ServiceClient& coord_client, par_kinematics::coord& coords,
			  par_trajectory_planning::commands& cmd)
{
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
}

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
        
        push_angles(coord_client, coords, cmd);
        
        i++;
    }
}

void pick_configuration_file(const boost::filesystem::path& directory, std::string& file)
{
    std::vector<std::string> files;
    int i = 1;
    
    std::cout << "Please pick your preferred configuration file: " << std::endl;
    if ( boost::filesystem::exists( directory ) )
    {
        boost::filesystem::directory_iterator end;
        for( boost::filesystem::directory_iterator iter(directory);  iter != end; ++iter )
        {
            if (! boost::filesystem::is_directory( *iter ) )
            {
                std::cout << "[" << i << "]" << " " << iter->path().file_string() << std::endl;
                files.push_back( iter->path().file_string() );
            }
        }
    }
    std::cin >> i;
    file = files[i-1];
}

int menu()
{
    std::cout << "[" << MENU_INIT_COMM      << "] init communication"           << std::endl;
    std::cout << "[" << MENU_INIT_MOTOR     << "] init motors"                  << std::endl;
    std::cout << "[" << MENU_CONF_SIN_MOT   << "] configure single abs motion"  << std::endl;
    std::cout << "[" << MENU_CONF_PTP_MOT   << "] configure PTP motion"         << std::endl;
    std::cout << "[" << MENU_RD_CONF_FILE   << "] read configuration file"      << std::endl;
    std::cout << "[" << MENU_START_MOT      << "] start motion"                 << std::endl;
    std::cout << "[" << MENU_START_HM	    << "] start homing"			        << std::endl;
    std::cout << "[" << MENU_START_TEST     << "] test"                         << std::endl;
    std::cout << "[" << MENU_EXIT           << "] exit"                         << std::endl;
    
    int x = 0;
    std::cin >> x;
    return x;
}
