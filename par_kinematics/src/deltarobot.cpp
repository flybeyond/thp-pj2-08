#include <par_kinematics/deltarobot.h>

DeltaRobot::DeltaRobot(double hipLength, double ankleLength, double baseSize, double effectorSize) 
{
		hip = hipLength;
		ankle = ankleLength;
		base = baseSize;
		effector = effectorSize;
		effLoc = new Point(0,0,0);
    
		newArms();
}
  
void DeltaRobot::newArms() 
{
	for (int i = 0; i < 3; i++) {
		m[i]   = new ArmModel(i*120, hip, ankle, base, effector);
		arm[i] = new DeltaArm(m[i]);
	}

	servoMin = Util::rad(-6.5);
	servoMax = 0;
}
  
bool DeltaRobot::isEffectorOk() 
{
	
	return !(Util::isNaN(m[0]->angle) || Util::isNaN(m[1]->angle) || Util::isNaN(m[2]->angle));
}
  
bool DeltaRobot::moveto(Point goal) 
{
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
