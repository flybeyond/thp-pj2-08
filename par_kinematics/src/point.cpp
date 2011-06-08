/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 *
 * This code is based on http://sensi.org/~svo/dbot/applet/ by Viacheslav Slavinsky. 
 */

#include <par_kinematics/point.h>

Point::Point(double x, double y, double z) 
{
	moveto(x,y,z);
}

Point Point::moveto(double x, double y, double z) 
{
	this->x = x;
	this->y = y;
	this->z = z;

	return (*this);
}

Point Point::moveto(Point o) 
{
	this->x = o.x;
	this->y = o.y;
	this->z = o.z;

	return Point(x, y, z);
}

Point Point::offset(Point o) 
{
	this->x += o.x;
	this->y += o.y;
	this->z += o.z;

	return Point(x, y, z);
}

double Point::distFrom(Point o) 
{
	return Util::dist(this->x, this->y, this->z, o.x, o.y, o.z);
}

Point::Point(const Point& p) 
{
	this->x = p.x;
	this->y = p.y;
	this->z = p.z;
}

void Point::rotateY(double phi) 
{
	double x2 = x*cos(phi) - z*sin(phi);
	double z2 = x*sin(phi) + z*cos(phi);

	x = x2;
	z = z2;
}		

void Point::rotateZ(double phi) 
{
	double x2 = x*cos(phi) - y*sin(phi);
	double y2 = x*sin(phi) + y*cos(phi);

	x = x2;
	y = y2;
}
