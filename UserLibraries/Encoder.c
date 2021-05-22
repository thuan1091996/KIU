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

#ifndef USERLIBRARIES_ENCODER_C_
#define USERLIBRARIES_ENCODER_C_

/******************************************************************************
* Includes
*******************************************************************************/
#include "Encoder.h"

/******************************************************************************
* Defines
*******************************************************************************/


/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/

// *****************************************************************************
//
//! QEI Initialization.
//! QEI Quadrature mode, no swap, no index, no Filter
//! QEI Velocity Initialization with Timer clock = System clock
//! Setting for QEI Timer Expire interrupt to interrupt every period
//! \param:  ui8Priority0        :QEI Int handler priority
//
//******************************************************************************
void QEI0_Init(uint8_t ui8Priority0)
{
    //Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));
    //Unlock PD7
    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;

    //GPIO Configure for QEI
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6);
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);

    //QEI Configure
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE, (QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));
    QEIVelocityDisable(QEI0_BASE);
    QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP, 0);
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, (SysCtlClockGet() / QEI0_INT_FREQ));

    //Filter
    QEIFilterConfigure(QEI0_BASE, QEI_FILTCNT_10);
    QEIFilterEnable(QEI0_BASE);

    //Interrupt configure
    QEIIntRegister(QEI0_BASE, QEI0_INTHandler);
    IntPrioritySet(INT_QEI0_TM4C123, ui8Priority0);
    QEIPositionSet(QEI0_BASE, 0);       //Reset counter

    //Enable QEI
    QEIVelocityEnable(QEI0_BASE);
    QEIEnable(QEI0_BASE);
    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);  //Velocity timer expires interrupt enable

};

void QEI1_Init(uint8_t ui8Priority1)
{
    //Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1));

    //GPIO Configure for QEI
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);

    //QEI Configure
    QEIDisable(QEI1_BASE);
    QEIIntDisable(QEI1_BASE, (QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));
    QEIVelocityDisable(QEI1_BASE);
    QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP, 0);
    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, (SysCtlClockGet() / QEI1_INT_FREQ));

    //Filter
    QEIFilterConfigure(QEI1_BASE, QEI_FILTCNT_10);
    QEIFilterEnable(QEI1_BASE);

    //Interrupt configure
    QEIIntRegister(QEI1_BASE, QEI1_INTHandler);
    IntPrioritySet(INT_QEI1_TM4C123, ui8Priority1);
    QEIPositionSet(QEI1_BASE, 0);

    //Enable QEI
    QEIEnable(QEI1_BASE);
    QEIVelocityEnable(QEI1_BASE);
    QEIIntEnable(QEI1_BASE, QEI_INTTIMER);  //Velocity timer expires interrupt enable

};


/* -----------Update_Position ---------------
 * Update position by accumulated position overtime
 * Input: Pointer to variable contain "Current Position"
 * Output: No
 */
void Update_Position1(float *Pt_Pos1)
{
    static uint32_t pos_temp=0;                     //Falling/Rising edges detected in current period
    static int8_t   dir_temp=0;                     //Direction of rotation
    float pos_offset=0;                             //Position offset
    dir_temp=QEIDirectionGet(QEI0_BASE);
    pos_temp=QEIPositionGet(QEI0_BASE);
    if(dir_temp==1)                                 //CW direction
    {
        if(pos_temp!=0)
        {
            QEIPositionSet(QEI0_BASE, 0);           //Reset position
            pos_offset=(pos_temp*360.0)/(ENCODER0_RESOLUTION*1.0);
            *Pt_Pos1 +=pos_offset;
        }
    }
    else if(dir_temp==-1)                           //CCW direction
    {
        if(pos_temp!=0)
        {
            QEIPositionSet(QEI0_BASE, 0);           //Reset position
            pos_offset=360.0-((pos_temp*360.0)/(ENCODER1_RESOLUTION*1.0));
            *Pt_Pos1 -=pos_offset;
        }
    }
};

// *****************************************************************************
//
//! Update Velocity of encoder 0
//
//******************************************************************************
void Update_Velocity0(int16_t *Pt_Vel0)
{
    int16_t prev_speed  = *Pt_Vel0;
    int16_t delta_speed;
#ifdef CAL_OPTIMIZEED_WITH_FIXED_FREQ
    int32_t cur_speed   = (QEIVelocityGet(QEI0_BASE) * 60) / 132 ; /* RPM = (pulses * int_freq * 60) / resolution  */
#else
    int32_t cur_speed   = (QEIVelocityGet(QEI0_BASE) * QEI0_INT_FREQ * 60) / ENCODER0_RESOLUTION ;
#endif  /* End of ifdef CAL_OPTIMIZEED_WITH_FIXED_FREQ */

    cur_speed *= QEIDirectionGet(QEI0_BASE);

    if (cur_speed > prev_speed)
    {
        delta_speed = cur_speed - prev_speed;
    }
    else
    {
        delta_speed = prev_speed - cur_speed;
    }

    /* Filter if changes way too much */
    if(delta_speed <= MOTOR0_MAX_SPEED)
    {
        *Pt_Vel0 = (int16_t)cur_speed;
    }
};

// *****************************************************************************
//
//! Update Velocity of encoder 1
//
//******************************************************************************
void Update_Velocity1(int16_t *Pt_Vel1)
{
    int16_t prev_speed  = *Pt_Vel1;
    int16_t delta_speed;
#ifdef CAL_OPTIMIZEED_WITH_FIXED_FREQ
    int32_t cur_speed   = (QEIVelocityGet(QEI1_BASE) * 60) / 132 ; /* RPM = (pulses * int_freq * 60) / resolution  */
#else
    int32_t cur_speed   = (QEIVelocityGet(QEI1_BASE) * QEI1_INT_FREQ * 60) / ENCODER1_RESOLUTION ;
#endif  /* End of ifdef CAL_OPTIMIZEED_WITH_FIXED_FREQ */
    cur_speed *= QEIDirectionGet(QEI1_BASE);
    if (cur_speed > prev_speed)
    {
        delta_speed = cur_speed - prev_speed;
    }
    else
    {
        delta_speed = prev_speed - cur_speed;
    }
    /* Filter if changes way too much */
    if(delta_speed <= MOTOR1_MAX_SPEED)
    {
        *Pt_Vel1 = (int16_t)cur_speed;
    }
};


// *******************************************************************************
//
//! QEIx Timer Expire INT Handler
//
//********************************************************************************

__attribute__((__weak__)) void QEI0_INTHandler(void)
{
    QEIIntClear(QEI0_BASE, QEI_INTTIMER);       //acknowledge interrupt
};


__attribute__((__weak__)) void QEI1_INTHandler(void)
{
    QEIIntClear(QEI1_BASE, QEI_INTTIMER);       //acknowledge interrupt
};


#endif /* USERLIBRARIES_ENCODER_C_ */
