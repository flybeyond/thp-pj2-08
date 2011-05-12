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

/** Three DeltaArms, the base and the effector. */
class DeltaRobot {
public:
	ArmModel* m[3];
	DeltaArm* arm[3];
  
	//  private DeltaArm arm0, arm120, arm240;
	//  public double angle1, angle2, angle3;      // servo angles
	//  public double pangle1, pangle2, pangle3;   // parallelogram joint angles
  
	double servo[3];
	double pangle[3];
  
	double servoMin, servoMax;
  
	double hip, ankle, base, effector;
  
	Point* effLoc;
  
	DeltaRobot(double hipLength, double ankleLength, double baseSize, double effectorSize) {
		hip = hipLength;
		ankle = ankleLength;
		base = baseSize;
		effector = effectorSize;
		effLoc = new Point(0,0,0);
    
		newArms();
	}
  
	void newArms() {
		for (int i = 0; i < 3; i++) {
			m[i]   = new ArmModel(i*120, hip, ankle, base, effector);
			arm[i] = new DeltaArm(m[i]);
		}
    
		servoMin = Util::rad(-6.5);
		servoMax = 0;
	}
  
	bool isEffectorOk() {
		
		return !(Util::isNaN(m[0]->angle) || Util::isNaN(m[1]->angle) || Util::isNaN(m[2]->angle));
	}
  
	bool moveto(Point goal) {
		for (int i = 0; i < 3; i++) {
			m[i]->moveto(goal.x, goal.y, goal.z);
			servo[i] = m[i]->rho;
			pangle[i] = m[i]->pangle;
		}

		effLoc->moveto(goal);
    
		servoMin = std::min( std::min(servo[0], servo[1]), servoMin); 
		servoMin = std::min(servo[2], servoMin);
		servoMax = std::max( std::max(servo[0], servo[1]), servoMax); 
		servoMax = std::max(servo[2], servoMax);
    
		return isEffectorOk();
	}
  
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
