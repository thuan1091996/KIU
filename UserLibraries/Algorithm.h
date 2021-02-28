/****************************************************************************
* Title                 :   PID Implementation
* Filename              :   Algorithm.h
* Author                :   ItachiThuan
* Origin Date           :   Jul 8, 2019
* Version               :   1.0.1
* Target                :   TM4C123 with CCS IDE
*****************************************************************************/

/*************** INTERFACE CHANGE LIST **************************************
*    Date           + Software Version  +  Initials Description
*  Jul 8th, 2019      | v1.0.0            |  Interface Created.
*  Jan 1st, 2021      | v1.0.1            |  Re-config to more universally
*****************************************************************************/
/** \Algorithm
 *  \brief: Contains API control a plant with PID controller
 */
#ifndef USERLIBRARIES_ALGORITHM_H_
#define USERLIBRARIES_ALGORITHM_H_
/******************************************************************************
* Includes
*******************************************************************************/


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
/**
 * This constant is
 */


/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/



/******************************************************************************
* Typedefs
*******************************************************************************/
typedef struct
{
    float p_gain;
    float i_gain;
    float d_gain;
    float integrate_state;
    float integrate_max;
    float integrate_min;
    float prev_err;
    float drive_max;
    float drive_min;
}PID_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
#if REFACTOR
int8_t  PID_Controller(float Kp,
                       float Ki,
                       float Kd,
                       int16_t Error,
                       int16_t CurrentState);
#else
float UpdatePID(PID_t* pid_t, float error);
#endif /*End of REFACTOR*/




#endif /* USERLIBRARIES_ALGORITHM_H_ */
