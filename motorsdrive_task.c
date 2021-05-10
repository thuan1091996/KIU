/*******************************************************************************
* Title                 :   Motors Drive Task
* Filename              :   motorsdrive_task.c
* Author                :   Itachi
* Origin Date           :   03/01/2021
* Version               :   1.0.0
* Compile with          :   FreeRTOS V8.2.3
* Target                :   TM4C123
* Notes                 :   None
*******************************************************************************/

/*************** MODULE REVISION LOG ******************************************
*
*    Date    Software Version    Initials   Description
*  03/01/21     1.0.0             Itachi      Module initialized
*
*******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "UserLibraries/BridgeH.h"
#include "motorsdrive_task.h"
//*****************************************************************************
//
// The stack size for the Motors drive task.
//
//*****************************************************************************
#define DRIVEMOTORSSTACKSIZE        64         // Stack size in words


//*****************************************************************************
//
// Motor new data semaphore
//
//*****************************************************************************
//xQueueHandle g_pLEDQueue; //TODO: FIXME

volatile int8_t g_i8motor0speed=50;
volatile int8_t g_i8motor1speed=30;



//extern xSemaphoreHandle g_pUARTSemaphore; //TODO: FIXME
//*****************************************************************************
//
// This task is used to change speed of the motors
//
//*****************************************************************************
//TODO: FIXME
static void
DriveMotors(void *pvParameters)
{
    while(1)
    {
    /* Pend on semaphore
         * If the speed of motor0 change
         *  => Update motor0 speed
         * else if the speed of motor1 change
         *  => Update motor 1 speed
       Post on semaphore*/
        Motor0_UpdateSpeed(g_i8motor0speed);
        Motor1_UpdateSpeed(g_i8motor1speed);
        vTaskDelay(10);
    }
}

//*****************************************************************************
//
// Initializes the LED task.
//
//*****************************************************************************
uint32_t
DriveMotorsTaskInit(void)
{
    BaseType_t status;
    //
    // Initialization for motors
    //
    BridgeH_GPIO_Init();
    BridgeH_PWM_Init();
    status = xTaskCreate( (TaskFunction_t)       DriveMotors,
                          ( const char * const) "DriveMotors",
                          (uint16_t)             DRIVEMOTORSSTACKSIZE,
                          NULL,
                          tskIDLE_PRIORITY + PRIORITY_DRIVEMOTOR_TASK,
                          NULL);
    if(status != pdPASS)
    {
        while(1) {} //FIXME
    }
    return status;  /* Success */

}
