#include <ros/ros.h>
#include <par_trajectory_planning/motor.h>
#include <modbus/modbus.h>

#include <iostream>
#include <cmath>

#ifndef _STEPPERMOTOR_H_
#define _STEPPERMOTOR_H_

class StepperMotor : public Motor
{
    public:
        /**
         * @brief Destructor.
         */        
        virtual ~StepperMotor() { free(ctx); }
        /**
         * @brief Function for initializing the motor.
         */      
        virtual void init();
        /**
         * @brief This function starts the PTP motion.
         * @param x X coordinate in radians
         * @param y Y coordinate in radians.
         * @param z Z coordinate in radians.
         * @return bool True on success, false on failure. 
         */      
        bool startPTPMotion(double x, double y, double z);
    private:
        modbus_t* ctx;
};
#endif
