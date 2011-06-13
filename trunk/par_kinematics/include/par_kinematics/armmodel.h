/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 *
 * This code is based on http://sensi.org/~svo/dbot/applet/ by Viacheslav Slavinsky. 
 */

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <par_kinematics/util.h>
#include <par_kinematics/point.h>

#ifndef _ARMMODEL_H_
#define _ARMMODEL_H_

class ArmModel
{
	public:
		float alpha, beta, rho;
		double a, b; // ankle length, hip length
		double c;	 // base to goal distance, a2: projected ankle length

		Point* ab;

		double base;        // base size
		double effector;    // effector size
		double angle;		// this arm length
	
		Point* g;           // goal point 

		double pangle;      // parallelogram joint angle
		double ankleAngle;  // for transforms
		
		ArmModel(double angle, double hip, double ankle, double base, double effector);
		ArmModel(const ArmModel& p);
		double moveto(double xg, double yg, double zg);
};

#endif
