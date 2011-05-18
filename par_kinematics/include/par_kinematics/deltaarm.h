/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 *
 * This code is based on http://sensi.org/~svo/dbot/applet/ by Viacheslav Slavinsky. 
 */

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <par_kinematics/armmodel.h>
#include <par_kinematics/point.h>

#ifndef _DELTAARM_H_
#define _DELTAARM_H_

class DeltaArm {
	public:
		ArmModel* m;
  
		double awidth;  // ankle parallelogram width
		double goodRho; // last known possible angle
		double servo;   // servo disk size
  
		DeltaArm(ArmModel* m) {
		this->m = m;
		this->awidth = 40;
		this->servo = 15;
		}
  
		void setAnkleWidth(double awidth) {
			this->awidth = awidth;
		}
  
		double ankleWidth() { return this->awidth; }

};

#endif
