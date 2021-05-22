/****************************************************************************
* Title                 :   Mobile Robot application header
* Filename              :   app.c
* Author                :   ItachiThuan
* Origin Date           :   May 15, 2021
* Version               :   1.0.0
* Target                :   TM4C123 with CCS IDE
*****************************************************************************/

/*************** MODULE REVISION LOG**************************************
*    Date           + Software Version  +  Initials Description
*  May 15, 2021       | v1.0.0            |  Interface Created.
*****************************************************************************/

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "led_task.h"
#include "switch_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "app.h"

#include "UserLibraries/BridgeH.h"
#include "UserLibraries/Encoder.h"
/******************************************************************************
* Module pre-processor
*******************************************************************************/
#define QEI0_ISR_PRIORITY           5
#define QEI1_ISR_PRIORITY           5


/******************************************************************************
* Feature defines
*******************************************************************************/
#define FW_MAJORID                  0   //TODO: Update each release
#define FW_MINORID                  0
#define FW_CORRECTION_ID            1
#define HW_MAJOR_ID                 0
#define HW_MINOR_ID                 0
#define HW_CORRECTION_ID            0

#define TEST_TASK_MEM               0   //TODO: Reset to 0 when release
#define TEST_ENCODER                0   //TODO: Reset to 0 when release
/******************************************************************************
* Task defines
******************************************************************************/
// Task priorities: Higher -> More important
#define M0SPEEDUPDATE_PRIORITY      tskIDLE_PRIORITY + 7
#define M1SPEEDUPDATE_PRIORITY      tskIDLE_PRIORITY + 7
#define MOTORDRIVER_PRIORITY        tskIDLE_PRIORITY + 5

// Task stack size: In words (4-bytes)
#define MOTORDRIVER_STACK_DEPTH     64
#if TEST_ENCODER
#define M0SPEEDUPDATE_STACK_DEPTH   100
#define M1SPEEDUPDATE_STACK_DEPTH   100
#else
#define M0SPEEDUPDATE_STACK_DEPTH   50
#define M1SPEEDUPDATE_STACK_DEPTH   50
#endif  /* End of TEST_ENCODER */

#define TASK_TOTAL                  3


/******************************************************************************
* Macro
******************************************************************************/
/* The error routine that is called if the driver library encounters an error */
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
/******************************************************************************
* Module Typedefs
*******************************************************************************/
typedef struct
{
    TaskFunction_t const TaskCodePtr;           /*< Pointer to the task function */
    const char * const TaskName;                /*< String task name             */
    const uint16_t StackDepth;                  /*< Stack depth                  */
    void * const ParametersPtr;                 /*< Parameter Pointer            */
    UBaseType_t TaskPriority;                   /*< Task Priority                */
    TaskHandle_t * const TaskHandle;            /*< Pointer to task handle       */
}TaskInitParams_t;


/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void Task_MotorDriver(void *pvParameters);
static void Task_M0SpeedUpdate(void *pvParameters);
static void Task_M1SpeedUpdate(void *pvParameters);



void ConfigureUART(void);
/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
TaskHandle_t xHandlerM0SpeedUpdate = NULL;
TaskHandle_t xHandlerM1SpeedUpdate = NULL;
TaskHandle_t xHandlerMotorDriver = NULL;

xSemaphoreHandle g_pUARTSemaphore;  /* The mutex that protects concurrent access of UART from multiple tasks */

#if TEST_ENCODER
int16_t speed_m0=0, speed_m1=0;
int16_t g_est_speed0=50, g_est_speed1=50;
int8_t g_set_duty0=40, g_set_duty1=40;
#endif  /* End of TEST_ENCODER */


/* Task configuration table that contains all the parameters necessary to initialize the system tasks. */
TaskInitParams_t const TaskInitParameters[] =
{
 // Pointer Task function,  Task String Name,   The task stack, Parameter Pointer, Task priority  ,     Task Handle
   {&Task_MotorDriver,      "MotorDriver",    MOTORDRIVER_STACK_DEPTH,    NULL, MOTORDRIVER_PRIORITY,   &xHandlerMotorDriver},
   {&Task_M0SpeedUpdate,    "M0SpeedUpdater", M0SPEEDUPDATE_STACK_DEPTH,  NULL, M0SPEEDUPDATE_PRIORITY, &xHandlerM0SpeedUpdate},
   {&Task_M1SpeedUpdate,    "M1SpeedUpdater", M1SPEEDUPDATE_STACK_DEPTH,  NULL, M1SPEEDUPDATE_PRIORITY, &xHandlerM1SpeedUpdate},
};


//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run at 50 MHz from the PLL.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //
    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    //
    ConfigureUART();

    //
    // Print demo introduction.
    //
    UARTprintf("Autonomous mobile robot running FreeRTOS demo! \n");
    UARTprintf("System clock:  %d MHz! \n", (SysCtlClockGet() / 1000000));

    //
    // Initialization for motors
    //
    BridgeH_GPIO_Init();
    BridgeH_PWM_Init();
    UARTprintf("Bridge-H was initialized successfully \n");

    //
    // Initialization of encoder
    //
    QEI0_Init(QEI0_ISR_PRIORITY);
    UARTprintf("Encoder0 driver was initialized successfully \n");
    QEI1_Init(QEI1_ISR_PRIORITY);
    UARTprintf("Encoder1 driver was initialized successfully \n");

    //
    // Create a mutex to guard the UART.
    //
    g_pUARTSemaphore = xSemaphoreCreateMutex();

    //
    // Create the LED task.
    //
    if(LEDTaskInit() != 0)
    {
        for(;;)
        {
        }
    }

    //
    // Create the switch task.
    //
    if(SwitchTaskInit() != 0)
    {

        for(;;)
        {
        }
    }


    // Loop through the task table and create each task.
    int TaskCount;
    for (TaskCount = 0; TaskCount < TASK_TOTAL; ++TaskCount)
    {
        BaseType_t xstatus = xTaskCreate(TaskInitParameters[TaskCount].TaskCodePtr,
                                         TaskInitParameters[TaskCount].TaskName,
                                         TaskInitParameters[TaskCount].StackDepth,
                                         TaskInitParameters[TaskCount].ParametersPtr,
                                         TaskInitParameters[TaskCount].TaskPriority,
                                         TaskInitParameters[TaskCount].TaskHandle);
        if(xstatus != pdPASS)
        {
            UARTprintf("Creation of %s failed \n", TaskInitParameters[TaskCount].TaskName);
            for(;;)
            {

            }
        }
    }

    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //

    for(;;)
    {
    }
}


/******************************************************************************
* Function Definitions
*******************************************************************************/


//*****************************************************************************
//
// This task is used to drive the speed of the motors
//
//*****************************************************************************
static void
Task_MotorDriver(void *pvParameters)
{
    for(;;)
    {
    /* Pend on semaphore
         * If the speed of motor0 change
         *  => Update motor0 speed
         * else if the speed of motor1 change
         *  => Update motor 1 speed
       Post on semaphore*/
//        UARTprintf("Current motor 0 speed: %d \n", g_set_duty0);
//        UARTprintf("Current motor 1 speed: %d \n", g_set_duty1);
        #if TEST_ENCODER
        Motor0_UpdateSpeed(g_set_duty0);
        Motor1_UpdateSpeed(g_set_duty1);
        #endif  /* End of TEST_ENCODER */
        #if TEST_TASK_MEM
        UARTprintf("Motordrive Stack left %d \n", uxTaskGetStackHighWaterMark(xHandlerMotorDriver));
        #endif  /* End of TEST_TASK_MEM */
        vTaskDelay(10);
    }
}

//*****************************************************************************
//
// This task is used to get the new speed of the motor
//
//*****************************************************************************
static void
Task_M0SpeedUpdate(void *pvParameters)
{
    for(;;)
    {
        #if !TEST_ENCODER
        int16_t speed_m0;
        Update_Velocity0(&speed_m0);
        #else
        Update_Velocity0(&speed_m0);
        if (speed_m0 < g_est_speed0)
        {
            UARTprintf("Failed: cur_speed0: %d - min_est_speed1: %d \n", speed_m0, g_est_speed0);
        }
        #endif  /* End of !TEST_ENCODER */
        #if TEST_TASK_MEM
        UARTprintf("Motor 0 Speed update Stack left %d \n", uxTaskGetStackHighWaterMark(xHandlerM0SpeedUpdate));
        #endif  /* End of TEST_TASK_MEM */
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); /* Clear the notification value and block indefinitely */

    }
}

static void
Task_M1SpeedUpdate(void *pvParameters)
{
    for(;;)
    {
        #if !TEST_ENCODER
        int16_t speed_m1;
        Update_Velocity1(&speed_m1);
        #else
        Motor1_UpdateSpeed(g_set_duty1);
        Update_Velocity1(&speed_m1);
        if (speed_m1 < g_est_speed1)
        {
            UARTprintf("Failed: cur_speed1: %d - min_est_speed1: %d \n", speed_m1, g_est_speed1);
        }
        #endif  /* End of !TEST_ENCODER */
        #if TEST_TASK_MEM
        UARTprintf("Motor 1 Speed update Stack left %d \n", uxTaskGetStackHighWaterMark(xHandlerM1SpeedUpdate));
        #endif  /* End of TEST_TASK_MEM */
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); /* Clear the notification value and block indefinitely */
    }
}

void QEI0_INTHandler(void)
{
    QEIIntClear(QEI0_BASE, QEI_INTTIMER);       //acknowledge interrupt
    BaseType_t xHigherPriorityTaskWoken;

    /* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
        it will get set to pdTRUE inside the interrupt safe API function if a
        context switch is required. */
    xHigherPriorityTaskWoken = pdFALSE;

    /* Send a notification directly to the handler task. */
    vTaskNotifyGiveFromISR(xHandlerM0SpeedUpdate, &xHigherPriorityTaskWoken );

    /* Pass the xHigherPriorityTaskWoken value into portYIELD_FROM_ISR().  If
        xHigherPriorityTaskWoken was set to pdTRUE inside vTaskNotifyGiveFromISR()
        then calling portYIELD_FROM_ISR() will request a context switch.  If
        xHigherPriorityTaskWoken is still pdFALSE then calling
        portYIELD_FROM_ISR() will have no effect. */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
};

void QEI1_INTHandler(void)
{
    QEIIntClear(QEI1_BASE, QEI_INTTIMER);       //acknowledge interrupt
    BaseType_t xHigherPriorityTaskWoken;

    /* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
        it will get set to pdTRUE inside the interrupt safe API function if a
        context switch is required. */
    xHigherPriorityTaskWoken = pdFALSE;

    /* Send a notification directly to the handler task. */
    vTaskNotifyGiveFromISR(xHandlerM1SpeedUpdate, &xHigherPriorityTaskWoken );

    /* Pass the xHigherPriorityTaskWoken value into portYIELD_FROM_ISR().  If
        xHigherPriorityTaskWoken was set to pdTRUE inside vTaskNotifyGiveFromISR()
        then calling portYIELD_FROM_ISR() will request a context switch.  If
        xHigherPriorityTaskWoken is still pdFALSE then calling
        portYIELD_FROM_ISR() will have no effect. */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
};

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    for(;;)
    {
    }
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

