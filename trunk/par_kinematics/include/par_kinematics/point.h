#include <ros/ros.h>
#include <iostream>
#include <string>
#include <cmath>
#include <par_kinematics/util.h>

#ifndef _POINT_H_
#define _POINT_H_

/** Point in 3d space */
class Point {
	public: 
		double x, y, z;


		Point(double x, double y, double z) {
			moveto(x,y,z);
		}

		Point moveto(double x, double y, double z) {
			this->x = x;
			this->y = y;
			this->z = z;
	
			return (*this);
		}

		Point moveto(Point o) {
			this->x = o.x;
			this->y = o.y;
			this->z = o.z;

			return Point(x, y, z);
		}

		Point offset(Point o) {
			this->x += o.x;
			this->y += o.y;
			this->z += o.z;

			return Point(x, y, z);
		}

		double distFrom(Point o) {
			return Util::dist(this->x, this->y, this->z, o.x, o.y, o.z);
		}

		Point(const Point& p) {
			this->x = p.x;
			this->y = p.y;
			this->z = p.z;
		}

		void rotateY(double phi) {
			double x2 = x*cos(phi) - z*sin(phi);
			double z2 = x*sin(phi) + z*cos(phi);

			x = x2;
			z = z2;
		}		

		void rotateZ(double phi) {
			double x2 = x*cos(phi) - y*sin(phi);
			double y2 = x*sin(phi) + y*cos(phi);

			x = x2;
			y = y2;
		}

};

#endif
