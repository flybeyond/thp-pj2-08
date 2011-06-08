/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 *
 * This code is based on http://sensi.org/~svo/dbot/applet/ by Viacheslav Slavinsky. 
 */

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <par_kinematics/armmodel.h>
#include <par_kinematics/deltaarm.h>
#include <par_kinematics/point.h>
#include <par_kinematics/util.h>

#ifndef _DELTAROBOT_H_
#define _DELTAROBOT_H_

class DeltaRobot {
public:
	ArmModel* m[3];
	DeltaArm* arm[3];
	double servo[3];
	double pangle[3];
	double servoMin, servoMax;
	double hip, ankle, base, effector;
	Point* effLoc;
  
	DeltaRobot(double hipLength, double ankleLength, double baseSize, double effectorSize);
  
	void newArms();
  
	bool isEffectorOk();
  
	bool moveto(Point goal);
  
	void setHip(double b) { this->hip = b; newArms(); }
  
	double getHip() { return this->hip; }
  
	void setAnkle(double a) { this->ankle = a; newArms();}
  
	double getAnkle() { return this->ankle; }
  
	void setBase(double base) { this->base = base; newArms();}
  
	double getBase() { return this->base; }
  
	void setEffector(double effector) { this->effector = effector; newArms();}
  
	double getEffector() { return this->effector; }

};

#endif
