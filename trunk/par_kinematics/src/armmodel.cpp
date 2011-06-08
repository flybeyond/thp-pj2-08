/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 *
 * This code is based on http://sensi.org/~svo/dbot/applet/ by Viacheslav Slavinsky. 
 */

#include <par_kinematics/armmodel.h>

ArmModel::ArmModel(double angle, double hip, double ankle, double base, double effector) 
{
    this->angle = Util::rad(angle);
    this->base = base;
    this->effector = effector;
    b = hip;
    a = ankle;
    ab = new Point(0,0,0);
}
		
ArmModel::ArmModel(const ArmModel& p) 
{
    this->alpha = alpha;
    this->beta = beta;
    this->rho = rho;
    this->ab = ab;
    this->base = base;
    this->effector = effector;
    this->angle = angle;
    this->g = g;
    this->pangle = pangle;
    this->ankleAngle = ankleAngle;
    this->a = a;
    this->b = b;
    this->c = c;
}
			
double ArmModel::moveto(double xg, double yg, double zg) 
{
    g = new Point(xg, yg, zg);
    g->rotateY(-angle);
    // offset the goal by effector size
    g->offset(Point(effector,0,0));

    // find projection of a on the z=0 plane, squared
    double a2 = a*a - g->z*g->z;  //sqrt(a*a - g.z*g.z);

    // calculate c with respect to base offset
    // pure math version (for verification):
    // c = sqrt((xg+effector-base)*(xg+effector-base) + yg*yg);
    // but since we have g-spot offset by effector, we can use dist() 
    // to calculate c like this:

    c = Util::dist(g->x, g->y, 0, base, 0, 0);

    alpha = acos( (-a2+b*b+c*c)/(2*b*c));

    beta = atan2( g->y, g->x-base );

    rho = alpha - beta;

    ab->moveto(base+b*cos(rho), -b*sin(rho), 0);

    // parallelogram angles
    // angle between y-axis and ankle bone in the plane z=0
    ankleAngle = atan2(g->x-ab->x, g->y-ab->y);

    // create a point for end effector, offset by location AB
    Point pend = Point(g->x - ab->x, g->y - ab->y, g->z);
    // rotate its plane by ankleAngle
    pend.rotateZ(ankleAngle);
    // find the parallelogram joint angle
    pangle = atan2(pend.y, pend.z);

    return rho;
}
