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

class DeltaRobot 
{
    public:
        // public variable for servo angles.
        double servo[3];
        /**
         * @brief Constructor for deltarobot.
         * @param hipLength Length of hip.
         * @param ankleLength Length of ankle.
         * @param baseSize Size of base.
         * @param effectorSize Size of effector
         */
        DeltaRobot(double hipLength, double ankleLength, double baseSize, double effectorSize);
        /**
         * @brief Method for creating new delta robot arms.
         */
        void newArms();
        /**
         * @brief Method for checking the end effector.
         * @return TRUE if OK else FALSE.
         */
        bool isEffectorOk();
        /**
         * @brief Method for moving the delta robot to a specified point.
         * @param goal Point object with X, Y, Z defined.
         */
        bool moveto(Point goal);
        /**
         * @brief Method for setting the hip.
         * @param b Length of new hip.
         */
        void setHip(double b) { this->hip = b; newArms(); }
        /**
         * @brief Method for returning the hip length.
         * @return hip length.
         */
        double getHip() { return this->hip; }
        /**
         * @brief Method for setting the ankle.
         * @param a Length of new ankle.
         */
        void setAnkle(double a) { this->ankle = a; newArms();}
        /**
         * @brief Method for returning the ankle length.
         * @return ankle length.
         */
        double getAnkle() { return this->ankle; }
        /**
         * @brief Method for setting the base size.
         * @param base size of base.
         */
        void setBase(double base) { this->base = base; newArms();}
        /**
         * @brief Method for returning the base size.
         * @return base size.
         */
        double getBase() { return this->base; }
        /**
         * @brief Method for setting effector's size.
         * @param effector size of effector.
         */
        void setEffector(double effector) { this->effector = effector; newArms();}
        /**
         * @brief Method for getting the effector's size.
         * @return size of effector.
         */
        double getEffector() { return this->effector; }
private:
	ArmModel* m[3];
	DeltaArm* arm[3];
	double pangle[3];
	double servoMin, servoMax;
	double hip, ankle, base, effector;
	Point* effLoc;
};

#endif
