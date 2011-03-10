#include <ros/ros.h>

#include <iostream>
#include <cmath>

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

/**
 * Trigonometric constants.
 */
static const double sqrt3  = sqrt(3.0);
static const double pi     = 3.141592653;
static const double sin120 = sqrt3/2.0;   
static const double cos120 = -0.5;        
static const double tan60  = sqrt3;
static const double sin30  = 0.5;
static const double tan30  = 1/sqrt3;   

class Kinematics
{
    public: 
        /**
         * @brief Constructor.
         */
        Kinematics();
        
        /**
         * @brief This is the basic IK method that will compute and return an IK solution.
         * @param x0 X coordinate in degrees.
         * @param y0 Y coordinate in degrees.
         * @param z0 Z coordinate in degrees.
         * @return bool True on success, false on failure.
         */        
        bool getPositionIK(double x0, double y0, double z0);
        
        /**
         * @brief This methods returns theta 1 in degrees [X].
         * @return double theta 1, X-coordinate.
         */
        double getTheta1() { return theta1; }
        
        /**
         * @brief This methods returns theta 2 in degrees [Y].
         * @return double theta 1, Y-coordinate.
         */
        double getTheta2() { return theta2; }
        
        /**
         * @brief This methods returns theta 3 in degrees [Z].
         * @return double theta 1, Z-coordinate.
         */
        double getTheta3() { return theta3; }        
        
    private:
        /**
         * Robot geometry.
         */
        double e;     // effector
        double f;     // base
        double re;    // effector arm length
        double rf;    // base arm length
        
        /**
         * Inverse kinematics members.
         */
        double theta1;
        double theta2;
        double theta3;
        
        /**
         * Helper methods.
         */
        bool IK_calcAngleYZ(double x0, double y0, double z0, double &theta);
};

#endif
