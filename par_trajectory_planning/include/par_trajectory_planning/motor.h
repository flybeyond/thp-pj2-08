#include <ros/ros.h>

#include <iostream>
#include <cmath>

#ifndef _MOTOR_H_
#define _MOTOR_H_

class Motor
{
    public:
        /**
         * @brief Destructor.
         */
        virtual ~Motor() { }
        /**
         * @brief Virtual function for initializing the motor.
         */        
        virtual void init() = 0;
        /**
         * @brief Virtual function for starting the motor
         */
        virtual void start() = 0;
        /**
         * @brief Virtual function for stopping the motor
         */
        virtual void stop() = 0;
};

#endif
