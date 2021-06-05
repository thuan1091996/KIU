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
#include "UserLibraries/Algorithm.h"
/******************************************************************************
* Module pre-processor
*******************************************************************************/
#define QEI0_ISR_PRIORITY           5
#define QEI1_ISR_PRIORITY           5
#define UART_Q_LEN                  10
#define UART_MSG_MAX_LEN            100


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
#define TEST_MSG_Q                  0   //TODO: Reset to 0 when release
#define TEST_PID_M0                 1
#define TEST_PID_M1                 1
/******************************************************************************
* Task defines
******************************************************************************/
// Task priorities: Higher -> More important
#define M0SPEEDUPDATE_PRIORITY      tskIDLE_PRIORITY + 7
#define M1SPEEDUPDATE_PRIORITY      tskIDLE_PRIORITY + 7
#define M0CALPID_PRIORITY           tskIDLE_PRIORITY + 6
#define M1CALPID_PRIORITY           tskIDLE_PRIORITY + 6
#define OUTPUT_MSG_PRIORITY         tskIDLE_PRIORITY + 5
// Task stack size: In words (4-bytes)
#define OUTPUTMSG_STACK_DEPTH       200
#define M0CALPID_STACK_DEPTH        300
#define M1CALPID_STACK_DEPTH        300
#define M0SPEEDUPDATE_STACK_DEPTH   100
#define M1SPEEDUPDATE_STACK_DEPTH   100


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

typedef struct
{
    uint8_t DataLen;
    char*   PData;
}DataMsg_t;


/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void Task_OutputMSG(void *pvParameters);
static void Task_M0SpeedUpdate(void *pvParameters);
static void Task_M1SpeedUpdate(void *pvParameters);
static void Task_M0CalPID(void *pvParameters);
static void Task_M1CalPID(void *pvParameters);


void ConfigureUART(void);
/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
PID_t pid_m0 =
{
     .p_gain           = 0.37,
     .i_gain           = 0.116,
     .d_gain           = 0.12,
     .integrate_max    = 1000,
     .integrate_min    = -1000,
     .drive_max        = 100,
     .drive_min        =-100,
     .prev_err         = 0.0
};

PID_t pid_m1 =
{
     .p_gain           = 0.23,
     .i_gain           = 0.1,
     .d_gain           = 0.04,
     .integrate_max    = 1000,
     .integrate_min    = -1000,
     .drive_max        = 100,
     .drive_min        =-100,
     .prev_err         = 0.0
};

TaskHandle_t    xHandlerM0SpeedUpdate   = NULL;
TaskHandle_t    xHandlerM1SpeedUpdate   = NULL;
TaskHandle_t    xHandlerM0CalPID        = NULL;
TaskHandle_t    xHandlerM1CalPID        = NULL;
QueueHandle_t   xQueue_OutputMsg        = NULL;

SemaphoreHandle_t g_pUARTSemaphore;  /* The mutex that protects concurrent access of UART from multiple tasks */

#if TEST_ENCODER
int16_t speed_m0=0, speed_m1=0;
int16_t g_est_speed0=50, g_est_speed1=50;
#endif  /* End of TEST_ENCODER */

#if TEST_PID_M1
int16_t set_speed_m1=100;
int8_t  g_set_duty1=0;
#endif  /* End of TEST_PID_M1 */

#if TEST_PID_M0
int16_t set_speed_m0=100;
int8_t g_set_duty0=40;
#endif  /* End of TEST_PID_M0 */


/* Task configuration table that contains all the parameters necessary to initialize the system tasks. */
TaskInitParams_t const TaskInitParameters[] =
{
 // Pointer Task function,  Task String Name,   The task stack,     Parameter Pointer, Task priority  ,     Task Handle
   {&Task_M0SpeedUpdate,    "NewSpeedM0",       M0SPEEDUPDATE_STACK_DEPTH,  NULL, M0SPEEDUPDATE_PRIORITY,   &xHandlerM0SpeedUpdate},
   {&Task_M1SpeedUpdate,    "NewSpeedM1",       M1SPEEDUPDATE_STACK_DEPTH,  NULL, M1SPEEDUPDATE_PRIORITY,   &xHandlerM1SpeedUpdate},
   {&Task_M0CalPID,         "PIDM0",            M0CALPID_STACK_DEPTH,       NULL, M0CALPID_PRIORITY,        &xHandlerM0CalPID},
   {&Task_M1CalPID,         "PIDM1",            M1CALPID_STACK_DEPTH,       NULL, M1CALPID_PRIORITY,        &xHandlerM1CalPID},
   {&Task_OutputMSG,        "OutputMSG",        OUTPUTMSG_STACK_DEPTH,      NULL, OUTPUT_MSG_PRIORITY,      NULL},
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


    HWREG(DWT_BASE) |= 0x01;


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

    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_Start();


    //
    // Create a mutex to guard the UART.
    //
    g_pUARTSemaphore = xSemaphoreCreateMutex();
    if(g_pUARTSemaphore == NULL)
    {
        UARTprintf("Can't create UART mutex \n");
    }

    //
    // Create a message queue for output UART messages
    //
    xQueue_OutputMsg = xQueueCreate( UART_Q_LEN, sizeof(DataMsg_t) );
    if(xQueue_OutputMsg == NULL)
    {
        UARTprintf("Can't create UART message queue \n");
    }

    //
    // Create the LED task.
    //
    if(LEDTaskInit() != 0)
    {
        for(;;)
        {
        }
    }


    // Loop through the task table and create each task.
    int TaskCount = 0;
    for (; TaskCount < sizeof(TaskInitParameters) / sizeof(TaskInitParameters[0]); ++TaskCount)
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
    // In case the scheduler returns for some reason, print an error and loop forever.
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
// This task is used to get new duty1 value for motor1
//
//*****************************************************************************

static void
Task_M0CalPID(void *pvParameters)
{
    BaseType_t xStatus;
    int16_t speed_m0_recv=0;
    for(;;)
    {
        xStatus = xTaskNotifyWait(0, 0, (int16_t*)&speed_m0_recv, portMAX_DELAY);
        if(xStatus == pdPASS)
        {
            g_set_duty0 = (int8_t)UpdatePID(&pid_m0, set_speed_m0 - speed_m0_recv);
            Motor0_UpdateSpeed(g_set_duty0);
            #if TEST
            xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf("Speed received PID1: %d \n", speed_m1_recv);
            xSemaphoreGive(g_pUARTSemaphore);
            #endif
        }
        #if TEST_TASK_MEM
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Calculate PID motor 1 stack left: %d\n", uxTaskGetStackHighWaterMark(xHandlerM1CalPID));
        xSemaphoreGive(g_pUARTSemaphore);
        #endif  /* End of TEST_TASK_MEM */
    }
}

static void
Task_M1CalPID(void *pvParameters)
{
    BaseType_t xStatus;
    int16_t speed_m1_recv=0;
    for(;;)
    {
        xStatus = xTaskNotifyWait(0, 0, (int16_t*)&speed_m1_recv, portMAX_DELAY);
        if(xStatus == pdPASS)
        {
            g_set_duty1 = (int8_t)UpdatePID(&pid_m1, set_speed_m1 - speed_m1_recv);
            Motor1_UpdateSpeed(g_set_duty1);
            #if TEST
            xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf("Speed received PID1: %d \n", speed_m1_recv);
            xSemaphoreGive(g_pUARTSemaphore);
            #endif
        }
        #if TEST_TASK_MEM
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Calculate PID motor 1 stack left: %d\n", uxTaskGetStackHighWaterMark(xHandlerM1CalPID));
        xSemaphoreGive(g_pUARTSemaphore);
        #endif  /* End of TEST_TASK_MEM */
    }
}

//*****************************************************************************
//
// This task is used to print message via UART
//
//*****************************************************************************
static void
Task_OutputMSG(void *pvParameters)
{
    DataMsg_t xReceivedStructure;
    BaseType_t xStatus;
    char output_msg[UART_MSG_MAX_LEN]={0};
    for(;;)
    {
        xStatus = xQueueReceive( xQueue_OutputMsg, &xReceivedStructure, portMAX_DELAY);
        if(xStatus == pdPASS)
        {
            /* Get the message inside queue */
            memcpy(output_msg, xReceivedStructure.PData, xReceivedStructure.DataLen);
            /* Print message to UART */
            xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf("Data in output queue: %s", output_msg);
            xSemaphoreGive(g_pUARTSemaphore);
            /* Free up processed data */
            vPortFree(xReceivedStructure.PData);
            memset(output_msg, 0, UART_MSG_MAX_LEN);
        }
        #if TEST_TASK_MEM
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("OutputMSG Stack left %d \n", uxTaskGetStackHighWaterMark());
        xSemaphoreGive(g_pUARTSemaphore);
        #endif  /* End of TEST_TASK_MEM */
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
    int16_t speed_m0=0;
    for(;;)
    {
        #if !TEST_ENCODER
        Update_Velocity0(&speed_m0);
        #else
        Update_Velocity0(&speed_m0);
        Motor0_UpdateSpeed(g_set_duty0);
        if (speed_m0 < g_est_speed0)
        {
            xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf("Failed: cur_speed0: %d - min_est_speed1: %d \n", speed_m0, g_est_speed0);
            xSemaphoreGive(g_pUARTSemaphore);

        }
        #endif  /* End of !TEST_ENCODER */

        #if TEST_TASK_MEM
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Motor 0 Speed update Stack left %d \n", uxTaskGetStackHighWaterMark(xHandlerM0SpeedUpdate));
        xSemaphoreGive(g_pUARTSemaphore);
        #endif  /* End of TEST_TASK_MEM */
		xTaskNotify(xHandlerM0CalPID, speed_m0, eSetValueWithOverwrite); /* Send speed to to CalPID task */
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); /* Clear the notification value and block indefinitely */
    }
}

static void
Task_M1SpeedUpdate(void *pvParameters)
{
    int16_t speed_m1=0;
    for(;;)
    {
        #if !TEST_ENCODER
        Update_Velocity1(&speed_m1);
        #else
        Motor1_UpdateSpeed(g_set_duty1);
        Update_Velocity1(&speed_m1);
        if (speed_m1 < g_est_speed1)
        {
            xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
            UARTprintf("Failed: cur_speed1: %d - min_est_speed1: %d \n", speed_m1, g_est_speed1);
            xSemaphoreGive(g_pUARTSemaphore);
        }
        #endif  /* End of !TEST_ENCODER */

        #if TEST_TASK_MEM
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Motor 1 Speed update Stack left %d \n", uxTaskGetStackHighWaterMark(xHandlerM1SpeedUpdate));
        xSemaphoreGive(g_pUARTSemaphore);
        #endif  /* End of TEST_TASK_MEM */
        xTaskNotify(xHandlerM1CalPID, speed_m1, eSetValueWithOverwrite); /* Send speed to to CalPID task */
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
    // on entry to this function, so no processor interrupts will interrupt this loop.
    //
    char* taskname_derived = pcTaskGetTaskName(pxTask);
    char* taskname_return = pcTaskName;
    /* Re-check since data can be corrupted by overflowed */
    if (strcmp(taskname_derived, taskname_return) == 0)
    {
        UARTprintf("Task %s overflowed \n", taskname_return);
    }
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

