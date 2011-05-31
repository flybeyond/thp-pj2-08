/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 */

#include <ros/ros.h>
#include <par_kinematics/coord.h>
#include <par_trajectory_planning/commands.h>
#include <par_trajectory_planning/config.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#ifndef _UTILS_H_
#define _UTILS_H_

/**
 * @brief Function that asks kinematic model for motor angles.
 */			
void push_angles(ros::ServiceClient& coord_client, par_kinematics::coord& coords,
			  par_trajectory_planning::commands& cmd);
/**
 * @brief Function configuring a single motion.
 */				  
void configure_single_motion(par_trajectory_planning::commands& cmd);
/**
 * @brief Function for configuring PTP motions.
 */	
void configure_PTP_motion(ros::ServiceClient& coord_client, par_kinematics::coord& coords,
			  par_trajectory_planning::commands& cmd);
/**
 * @brief Function for choosing a configuration file.
 */
void pick_configuration_file(const boost::filesystem::path& directory, std::string& file);

/**
 * @brief The menu presented to the user.
 * @return int User's menu choice.
 */				  
int menu();

#endif
