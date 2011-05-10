#include <ros/ros.h>
#include <par_trajectory_planning/motor.h>
#include <par_trajectory_planning/commands.h>
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
         * @brief Function for starting the motor.
         */
        virtual void start();
        /**
         * @brief Function for stopping the motor.
         */
        virtual void stop();
        /**
         * @brief Function for initializing communication.
         */
        void initCom();        
        /** 
         * @brief Function for configuring single motions.
         */
        void confSingleMotion();
        /**
         * @brief Function for configuring PTP motions.
         */
        void confPTPMotion(const par_trajectory_planning::commands& cmd);
        /**
         *@brief Function for exiting.
         */
        void exit();
    private:
        modbus_t* ctx;
        int motions;
};
#endif
