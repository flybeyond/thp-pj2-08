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

/** Util */
class Util {
		public:
			static double deg(double rad) {
				return double(rad/PI*180);
			}

			static double rad(double deg) {
				return double(deg/180*PI);
			}

			static bool isNaN(double x) {
				return x != x;
			}

			static double dist(double x1, double y1, double z1, double x2, double y2, double z2)
			{
				return sqrt( pow( (x2 - x1), 2) + pow( (y2 - y1), 2) + pow( (z2 - z1), 2) );
			}
};

#endif
