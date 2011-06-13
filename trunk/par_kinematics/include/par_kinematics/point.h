/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 *
 * This code is based on http://sensi.org/~svo/dbot/applet/ by Viacheslav Slavinsky. 
 */

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <cmath>
#include <par_kinematics/util.h>

#ifndef _POINT_H_
#define _POINT_H_

class Point 
{
	public: 
		Point(double x, double y, double z);
		
		Point moveto(double x, double y, double z);

		Point moveto(Point o);
		
		Point offset(Point o);

		double distFrom(Point o);

		Point(const Point& p);

		void rotateY(double phi);	

		void rotateZ(double phi);

        double x, y, z;
};

#endif
