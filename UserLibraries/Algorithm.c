/****************************************************************************
* Title                 :   PID Implementation
* Filename              :   Algorithm.c
* Author                :   ItachiThuan
* Origin Date           :   Jul 8, 2019
* Version               :   1.0.1
* Target                :   TM4C123 with CCS IDE
*****************************************************************************/

/*************** INTERFACE CHANGE LIST **************************************
*    Date           + Software Version  +  Initials Description
*  Oct 18, 2019       | v1.0.0            |  Interface Created.
*  Jan 1st, 2021      | v1.0.1            |  Re-config to more universally
*****************************************************************************/

/******************************************************************************
* Includes
*******************************************************************************/
#include "Userlibs.h"
#include "Algorithm.h"

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
#if REFACTOR
int8_t  PID_Controller(float Kp,
                       float Ki,
                       float Kd,
                       int16_t Error,
                       int16_t CurrentState)
{
    static     int16_t intergrate_state=0;
    int16_t    intergrate_statemax=1000;
    int16_t    intergrate_statemin=-1000;
    int16_t    drive_temp;
    int8_t     pid_drive;
    intergrate_state+=Error;
    if(intergrate_state>intergrate_statemax)        intergrate_state=intergrate_statemax;
    else if (intergrate_state<intergrate_statemin)  intergrate_state=intergrate_statemin;
    drive_temp= (double)Error *Kp; //P term
    drive_temp+= (double)intergrate_state*Ki;

    //Bounded value
    if(drive_temp>100)
    {
        drive_temp=100;
    }
//    else if( (drive_temp>0) && (drive_temp<25) )
//    {
//        drive_temp=-25;
//    }
//    else if( (drive_temp<0) && (drive_temp>-25) )
//    {
//        drive_temp=0;
//    }
//    else if( (drive_temp<-25) && (drive_temp>-100) )
//    {
//        drive_temp=0;
//    }
    else if (drive_temp<-100)    drive_temp=-100;
    pid_drive=drive_temp;
    return pid_drive;
};
#else
float UpdatePID(PID_t* pid_t, float error)
{
    float p_part=0.0, i_part=0.0, d_part=0.0;
    float retval=0.0;
    // Calculate the proportional part
    p_part = pid_t->p_gain * error;

    // Calculate the integral state with bounded value to anti-wind up
    pid_t->integrate_state += error;
    if (pid_t->integrate_state > pid_t->integrate_max)
    {
        pid_t->integrate_state = pid_t->integrate_max;
    }
    else if (pid_t->integrate_state < pid_t->integrate_min)
    {
        pid_t->integrate_state = pid_t->integrate_min;
    }
    // Calculate the integrate part
    i_part = pid_t->i_gain * pid_t->integrate_state;

    // Calculate the derivative part
    d_part = pid_t->d_gain * (error - pid_t->prev_err);
    pid_t->prev_err = error;    /* Update prev_plant_status */
    retval = p_part + i_part + d_part;

    // Bounded drive value
    if (retval >= pid_t->drive_max)      retval = pid_t->drive_max;
    else if (retval <= pid_t->drive_min) retval = pid_t->drive_min;

    return retval;
}
#endif /*End of REFACTOR*/

