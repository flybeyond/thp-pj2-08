/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 */

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
        * @brief Constructor.
	    */
	    StepperMotor(modbus_t* cm) { ctx = cm; is_test = false; stop_motion = false; }
        /**
         * @brief Destructor.
         */        
        virtual ~StepperMotor() { /*free(ctx);*/ }
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
         * @param cmd Object with instructions.
         */
        void confSingleMotion(const par_trajectory_planning::commands& cmd);
        /**
         * @brief Function for configuring PTP motions.
         * @param cmd Object with instructions.
         */
        void confPTPMotion(const par_trajectory_planning::commands& cmd);
        /**
         * @brief Function for exiting.
         */
        void exit();
        /**
         * @brief Function for converting angles in degrees to motor positions.
         * @param x Angle in degrees.
         * @return n amount of motor steps.
         */
        uint16_t angleToStep(double x, bool& invalid_motion);
        /**
         * @brief Function for developing purposes. 
         */
        void test();
    private:
	    /**
	    * Helper function for initialization of single motion.
	    */
	    void initSingleMotion(int slave, uint16_t pos_lo, uint16_t pos_up, 
	            uint16_t acc_lo, uint16_t acc_up, uint16_t dec_lo, uint16_t dec_up, 
	            uint16_t speed_lo, uint16_t speed_up, uint16_t operating_mode, uint16_t seq_pos, 
	            int off);

        modbus_t* ctx;
        int motions;
        int repeat_motions;
        bool is_test;
	    bool stop_motion;
};
#endif
