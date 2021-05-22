/****************************************************************************
* Title                 :   QEI encoder driver
* ProductLink           :   tdhshop.com.vn/dong-co-dc-servo-giam-toc-ga37
* Filename              :   Encoder.c
* Author                :   ItachiThuan
* Origin Date           :   Jul 8, 2019
* Version               :   1.0.1
* Target                :   TM4C123 with CCS IDE
* Notes                 :
        |  PhA  |  PhB
 -------+-------+--------
QEI0    |  PD6  |  PD7
QEI1    |  PC5  |  PC6
*****************************************************************************/

/*************** INTERFACE CHANGE LIST **************************************
*    Date           + Software Version  +  Initials Description
*  Jul 8th, 2019      | v1.0.0            |  Interface Created.
*  Dec 5th, 2020      | v1.0.1            |  Change hardware pin to re-start the project
*****************************************************************************/
/** \Encoder.h
 *  \brief: Contains API to read motor speed via encoder interface
 */
#ifndef USERLIBRARIES_ENCODER_H_
#define USERLIBRARIES_ENCODER_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "Userlibs.h"

/******************************************************************************
* Configuration Constants
*******************************************************************************/
#define MOTOR0_MAX_SPEED            330
#define MOTOR1_MAX_SPEED            330
#define ENCODER0_RESOLUTION         1320
#define ENCODER1_RESOLUTION         1320
#define QEI0_INT_FREQ               10
#define QEI1_INT_FREQ               10
#define QEI0_PRIORITY               2
#define QEI1_PRIORITY               2
#if (QEI0_INT_FREQ != 10) || (QEI1_INT_FREQ != 10)
    #error "Change the velocity equation used to optimize calculation time inside Update_Velocityx()"
#else
    #define CAL_OPTIMIZEED_WITH_FIXED_FREQ
#endif


/******************************************************************************
* Function Prototypes
*******************************************************************************/
void QEI0_INTHandler(void);
void QEI1_INTHandler(void);
void QEI0_Init(uint8_t ui8Priority0);
void QEI1_Init(uint8_t ui8Priority1);
void Update_Position0(float *Pt_Pos0);
void Update_Position1(float *Pt_Pos1);
void Update_Velocity0(int16_t *Pt_Vel0);
void Update_Velocity1(int16_t *Pt_Vel1);
#endif /* USERLIBRARIES_ENCODER_H_ */
