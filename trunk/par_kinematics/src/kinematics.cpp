#include <par_kinematics/kinematics.h>

Kinematics::Kinematics()
{
        e      = 3.466 * 25.0;     
        f      = 3.466 * 48.0;     
        re     = 220.0;    
        rf     = 37.0;    
        theta1 = 0.0;
        theta2 = 0.0;
        theta3 = 0.0;
}

bool Kinematics::getPositionIK(double x0, double y0, double z0)
{
	theta1 = theta2 = theta3 = 0.0;
	bool status = IK_calcAngleYZ(x0, y0, z0, theta1);
	if (status) status = IK_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
	if (status) status = IK_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
	
	return status;
}

bool Kinematics::IK_calcAngleYZ(double x0, double y0, double z0, double &theta)
{
	double y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
	y0 -= 0.5 * 0.57735    * e;     // shift center to edge
	// z = a + b*y
	double a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
	double b = (y1-y0)/z0;
	// discriminant
	double d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
	if (d < 0) return false; // non-existing point
	double yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
	double zj = a + b*yj;
	theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
	return true;
}
