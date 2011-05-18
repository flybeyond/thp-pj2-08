/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 */

#include <ros/ros.h>
#include <par_kinematics/coord.h>
#include <par_trajectory_planning/commands.h>
#include <par_trajectory_planning/config.h>
#include <tinyxml/tinyxml.h>

#ifndef _CONFIG_UI_H_
#define _CONFIG_UI_H_

class Config
{
    public:
	   /**
        * @brief Constructor.
        * @param file Path to file.
        * @param coord_client Client to request coordinates from kinematics model. 
        * @param coords Object with XYZ parameters configured.
        * @param cmd Object used for configuring trajectory parameters.
        * @param chatter_pub_cmd Object used for communication 
	    */    
        Config(std::string& file, ros::ServiceClient& coord_client, par_kinematics::coord& coords,
			  par_trajectory_planning::commands& cmd, ros::Publisher& chatter_pub_cmd);
        /**
         * @brief Function for reading XML configuration file.
         */			  
        void read();
    private:
        /**
         * @brief Function for parsing communication parameters.
         */		
        void parse_xml_comm();
        /**
         * @brief Function for parsing motor parameters.
         */	        
        void parse_xml_init();
        /**
         * @brief Function for parsing single motion parameters.
         */	          
        void parse_xml_mot();
        /**
         * @brief Function for parsing PTP parameters.
         */	          
        void parse_xml_ptp();
        /**
         * @brief Function for parsing start motion parameters.
         */	          
        void parse_xml_start();
        
        std::string file;
        TiXmlElement* config;
        ros::ServiceClient coord_client;
        par_kinematics::coord coords;
        par_trajectory_planning::commands cmd;
        ros::Publisher chatter_pub_cmd;
};

#endif
