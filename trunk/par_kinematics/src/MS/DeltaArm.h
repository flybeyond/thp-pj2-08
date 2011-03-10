#ifndef _DELTAARM_H_
#define _DELTAARM_H_

#include <iostream>
#include <cmath>

#include "ArmModel.h"
#include "Point.h"

/* deze klas nog fixen...*/
class DeltaArm {
	public:
		ArmModel* m;
  
		double awidth;       // ankle parallelogram width
		double goodRho;           // last known possible angle
		double servo;  // servo disk size
  
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