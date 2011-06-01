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

void Config::get_value(TiXmlNode* node, const std::string& key, std::string& value)
{
    if (node != NULL)
    {
        std::string el;
        el = node->ValueStr();
        if (el == key)
        {
            value = node->FirstChild()->ValueStr();
        }
        else
        {
            value = "invalid";
        }
    }
}

void Config::parse_xml_ptp()
{
            TiXmlNode* node = config->FirstChild();
            std::string key;
            std::string value;
            
            if (node != NULL)
            {
                get_value(node, "repeat_motions", value);
                if (value != "invalid")
                {
                    cmd.repeat_motions = atoi( value.c_str() );
                }
                else
                {   
                    cmd.repeat_motions = 1;
                }
            }
            
            if (node != NULL)
            {
                node = node->NextSibling();
                get_value(node, "acceleration", value);
                if (value != "invalid")
                {
                    uint32_t acc = atoi( value.c_str() );
                    cmd.acc_up = 0xFFFF & acc;
                    cmd.acc_lo = 0xFFFF & (acc >> 16);
                }
                else
                {
                    cmd.acc_up = MOTOR_ACC_UP;
                    cmd.acc_lo = MOTOR_ACC_LO;
                }
            }
            
            if (node != NULL)
            {
                node = node->NextSibling();
                get_value(node, "deceleration", value);
                if (value != "invalid")
                {
                    uint32_t dec = atoi( value.c_str() );
                    cmd.dec_up = 0xFFFF & dec;
                    cmd.dec_lo = 0xFFFF & (dec >> 16);
                }
                else
                {
                    cmd.dec_up = MOTOR_DEC_UP;
                    cmd.dec_lo = MOTOR_DEC_LO;
                }
            }
            
            if (node != NULL)
            {
                node = node->NextSibling();
                get_value(node, "speed", value);
                if (value != "invalid")
                {
                    uint32_t speed = atoi( value.c_str() );
                    cmd.speed_up = 0xFFFF & speed;
                    cmd.speed_lo = 0xFFFF & (speed >> 16);
                }
                else
                {
                    cmd.speed_up = MOTOR_SPEED_UP;
                    cmd.speed_lo = MOTOR_SPEED_LO;
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

