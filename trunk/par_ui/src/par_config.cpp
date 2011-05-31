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

void Config::parse_xml_comm()
{
}

void Config::parse_xml_init()
{
            TiXmlElement* motor = config->FirstChildElement();
            
            uint16_t speed_up;
            uint16_t speed_lo;
            
            speed_up = atoi( motor->FirstChild()->ValueStr().c_str() );
            speed_lo = atoi( motor->FirstChild()->ValueStr().c_str() );
}

void Config::parse_xml_mot()
{
}

void Config::parse_xml_ptp()
{
            TiXmlNode* node = config->FirstChild();
            if (node != NULL)
            {
                std::string firstNode = node->ValueStr();
                if (firstNode == "repeat_motions")
                {
                    cmd.repeat_motions = atoi( node->FirstChild()->ValueStr().c_str() );
                }
                else
                {
                    cmd.repeat_motions = 1;
                }
            }
            
            for (TiXmlElement* row = config->FirstChildElement(); row;
                row = row->NextSiblingElement())
            {
                TiXmlNode* X = row->FirstChild("X");
                TiXmlNode* Y = row->FirstChild("Y");
                TiXmlNode* Z = row->FirstChild("Z");
                if (X != NULL && Y != NULL && Z != NULL)
                {
                    coords.request.x = atof( X->FirstChild()->ValueStr().c_str() );
                    coords.request.y = atof( Y->FirstChild()->ValueStr().c_str() );
                    coords.request.z = atof( Z->FirstChild()->ValueStr().c_str() );
                    push_angles(coord_client, coords, cmd);
                }
            }
}

void Config::parse_xml_start()
{
}

void Config::read(const std::string& file)
{
    cmd.abs_pos.clear();
    cmd.xyz_pos.clear();

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
            std::cout << "Number #: " << x << std::endl;
            switch(x)
            {
                case MENU_INIT_COMM:
                    cmd.option = MENU_INIT_COMM;
                    parse_xml_comm();
                    chatter_pub_cmd.publish(cmd);
                break;
                case MENU_INIT_MOTOR: 
                    cmd.option = MENU_INIT_MOTOR;
                    parse_xml_init();
                    chatter_pub_cmd.publish(cmd);
                break;
                case MENU_CONF_SIN_MOT:
                    cmd.option = MENU_CONF_SIN_MOT;
                    parse_xml_mot();
                    chatter_pub_cmd.publish(cmd);
                break;
                case MENU_CONF_PTP_MOT:
                    cmd.option = MENU_CONF_PTP_MOT;
                    parse_xml_ptp();
                    chatter_pub_cmd.publish(cmd);
                break;
                case MENU_START_MOT:
                    cmd.option = MENU_START_MOT;
                    parse_xml_start();
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

