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
	    StepperMotor(modbus_t* cm) { ctx = cm; is_test = false; }
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
         * @param cmd Object with instructions.
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
         * @return uint16_t n amount of motor steps.
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
	            uint16_t acc_up, uint16_t acc_lo, uint16_t dec_up, uint16_t dec_lo, 
	            uint16_t speed_up, uint16_t speed_lo, int off);

        modbus_t* ctx;
        int motions;
        int repeat_motions;
        bool is_test;
};
#endif
