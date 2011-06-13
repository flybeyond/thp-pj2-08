/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 */
 
#include <ros/ros.h>
#include <iostream>
#include <par_ui/par_utils.h>
#include <par_ui/par_config.h>
#include <par_kinematics/coord.h>
#include <par_trajectory_planning/commands.h>
#include <par_trajectory_planning/config.h>
#include <tinyxml/tinyxml.h>

Config::Config(ros::ServiceClient& coord_client, par_kinematics::coord& coords,
			  par_trajectory_planning::commands& cmd, ros::Publisher& chatter_pub_cmd)
{
    this->coord_client = coord_client;
    this->coords = coords;
    this->cmd = cmd;
    this->chatter_pub_cmd = chatter_pub_cmd;
}

void Config::parse_xml_init()
{
    /**
     * Implement this function for parsing motor parameters.
     */
}

void Config::parse_xml_ptp()
{                             
    for (TiXmlNode* node = config->FirstChild(); node;
        node = node->NextSibling() )
    {
        if (node->ValueStr() == "row")
        {
            uint16_t om = MOTOR_OPM_SINGLE;
            parse_xml_ptp_row(node, coords.request.x, coords.request.y, coords.request.z, om);
            push_angles(coord_client, coords, cmd);
            cmd.operating_mode.push_back( om );
        }
        else if (node->ValueStr() == "repeat_motions")
        {
            cmd.repeat_motions = atoi( node->FirstChild()->ValueStr().c_str() );
            std::cout << "repeat motions: " << cmd.repeat_motions << std::endl;
        }
        else if (node->ValueStr() == "acceleration")
        {
            uint32_t acc = atoi( node->FirstChild()->ValueStr().c_str() );
            cmd.acc_up = 0xFFFF & acc;
            cmd.acc_lo = 0xFFFF & (acc >> 16);
            std::cout << "acceleration: " << acc << std::endl;
        }
        else if (node->ValueStr() == "deceleration")
        {
            uint32_t dec = atoi( node->FirstChild()->ValueStr().c_str() );
            cmd.dec_up = 0xFFFF & dec;
            cmd.dec_lo = 0xFFFF & (dec >> 16);       
            std::cout << "deceleration: " << dec << std::endl;         
        }
        else if (node->ValueStr() == "operating_speed")
        {
            uint32_t speed = atoi( node->FirstChild()->ValueStr().c_str() );
            cmd.op_speed_up = 0xFFFF & speed;
            cmd.op_speed_lo = 0xFFFF & (speed >> 16);    
            std::cout << "operating speed: " << speed << std::endl;            
        }
        else if (node->ValueStr() == "startup_speed")
        {
            uint32_t speed = atoi( node->FirstChild()->ValueStr().c_str() );
            cmd.st_speed_up = 0xFFFF & speed;
            cmd.st_speed_lo = 0xFFFF & (speed >> 16);  
            std::cout << "startup speed: " << speed << std::endl;
        }
    }                
}

void Config::parse_xml_ptp_row(TiXmlNode* node, double& X, double& Y, double& Z, uint16_t& operating_mode)
{
    TiXmlNode* x  = node->FirstChild("X");
    TiXmlNode* y  = node->FirstChild("Y");
    TiXmlNode* z  = node->FirstChild("Z");
    TiXmlNode* OM = node->FirstChild("mode"); 
    
    X = atof( x->FirstChild()->ValueStr().c_str() );
    Y = atof( y->FirstChild()->ValueStr().c_str() );
    Z = atof( z->FirstChild()->ValueStr().c_str() );
    
    if (OM != NULL)
    {
        std::string key = OM->FirstChild()->ValueStr();
        if ( key == "single" )
        {
            operating_mode = MOTOR_OPM_SINGLE;
        }
        else if ( key == "link" )
        {
            operating_mode = MOTOR_OPM_LINK1;
        }
        else if ( key == "cp" )
        {
            operating_mode = MOTOR_OPM_LINK2;
        }
        else
        {
            operating_mode = MOTOR_OPM_SINGLE;
        }    
    }
}

void Config::read(const std::string& file)
{
    // don't forget this. if this is already filled with data and not cleared
    // behavior is undefined. 
    cmd.abs_pos.clear();
    cmd.xyz_pos.clear();
    cmd.operating_mode.clear();
    
    // set object defaults.
    cmd.repeat_motions = 1;
    cmd.acc_up = MOTOR_ACC_UP;
    cmd.acc_lo = MOTOR_ACC_LO;    
    cmd.dec_up = MOTOR_DEC_UP;
    cmd.dec_lo = MOTOR_DEC_LO;
    cmd.op_speed_up = MOTOR_OP_SPEED_UP;
    cmd.op_speed_lo = MOTOR_OP_SPEED_LO;    
    cmd.st_speed_up = MOTOR_ST_SPEED_UP;
    cmd.st_speed_lo = MOTOR_ST_SPEED_LO;        

	TiXmlDocument doc(file.c_str());
	std::cout << "file: " << file.c_str() << std::endl;
	
	if (doc.LoadFile())
	{
	    TiXmlElement *root = doc.RootElement();
        for(config = root->FirstChildElement(); config;
                config = config->NextSiblingElement()) 
        {
            int x = 0;
            config->QueryIntAttribute("number", &x);
            switch(x)
            {
                case MENU_INIT_MOTOR: 
                    cmd.option = MENU_INIT_MOTOR;
                    parse_xml_init();
                    chatter_pub_cmd.publish(cmd);
                break;
                case MENU_CONF_PTP_MOT:
                    cmd.option = MENU_CONF_PTP_MOT;
                    parse_xml_ptp();
                    chatter_pub_cmd.publish(cmd);
                break;
                case MENU_START_MOT:
                    cmd.option = MENU_START_MOT;
                    chatter_pub_cmd.publish(cmd);
                break;
            }
        }	    
	}
	else
	{
	    std::cout << "Could not load XML file." << std::endl;
	}
}

