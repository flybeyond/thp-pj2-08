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
        Config(ros::ServiceClient& coord_client, par_kinematics::coord& coords,
			  par_trajectory_planning::commands& cmd, ros::Publisher& chatter_pub_cmd);
        /**
         * @brief Function for reading XML configuration file.
         */			  
        void read(const std::string& file);
    private:
        /**
         * @brief Function for parsing motor parameters.
         */	        
        void parse_xml_init();
        /**
         * @brief Function for parsing PTP parameters.
         */	          
        void parse_xml_ptp();
        /**
         * @brief Function for parsing a PTP motion row.
         * @param node XML node to parse.
         * @param X X coordinate.
         * @param Y Y coordinate.
         * @param Z Z coordinate.
         * @param operating_mode The user defined operating mode.
         */
        void parse_xml_ptp_row(TiXmlNode* node, double& X, double& Y, double& Z, uint16_t& operating_mode);
        
        TiXmlElement* config;
        ros::ServiceClient coord_client;
        par_kinematics::coord coords;
        par_trajectory_planning::commands cmd;
        ros::Publisher chatter_pub_cmd;
};

#endif
