/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 *
 * This code is based on http://sensi.org/~svo/dbot/applet/ by Viacheslav Slavinsky. 
 */

#include <ros/ros.h>
#include <iostream>
#include <cmath>

#ifndef _UTIL_H_
#define _UTIL_H_

#define PI 3.14159265358979323846

class Util 
{
		public:
		    /**
		     * @brief Function for converting radians to degrees.
		     * @param rad Angle in radians.
		     * @return angle in degrees.
		     */
			static double deg(double rad) 
			{
				return double(rad/PI*180);
			}
            /**
             * @brief Function for converting degrees to radians.
             * @param deg Angle in degrees.
             * @return angle in radians.
             */
			static double rad(double deg) 
			{
				return double(deg/180*PI);
			}
            /**
             * @brief Function for checking validity of a number.
             * @param x A double value.
             * @return TRUE if x is not an number or FALSE otherwise.
             */
			static bool isNaN(double x) 
			{
				return x != x;
			}
            /**
             * @brief Function for calculating distance between 2 3D-points.
             * @param x1 Point x1.
             * @param y1 Point y1.
             * @param z1 Point z1.
             * @param x2 Point x2.
             * @param y2 Point y2.
             * @param z2 Point z2.
             * @return The calculated distance between the points.
             */
			static double dist(double x1, double y1, double z1, double x2, double y2, double z2)
			{
				return sqrt( pow( (x2 - x1), 2) + pow( (y2 - y1), 2) + pow( (z2 - z1), 2) );
			}
};

#endif
