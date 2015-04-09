#include <includes.h>
#include "main.h"
#include "app.h"

static  OS_STK          App_TaskStartStk[APP_TASK_START_STK_SIZE];
static  OS_STK          App_TaskSenSorStk[APP_TASK_SENSOR_STK_SIZE];
static  OS_STK          App_TaskUartStk[APP_TASK_UART_STK_SIZE];
static  OS_STK          App_TaskRunStk[APP_TASK_RUN_STK_SIZE];

static  OS_EVENT      *App_RUNMbox;
OS_EVENT      *App_StartMbox;
OS_EVENT      *App_UARTMbox;
//static  OS_EVENT      *App_RunMbox;
//-----------------------------

/**********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
**********************************************************************************************************/

static  void  App_TaskCreate             (void);
static  void  App_EventCreate            (void);

static  void  App_TaskStart              (void        *p_arg);
static  void  App_TaskSENSOR                (void        *p_arg);
static  void  App_TaskUart               (void        *p_arg);
static  void  App_TaskRun               (void        *p_arg);



/*static  void  App_FormatDec              (CPU_INT08U  *pstr,
                                          CPU_INT32U   value,
                                          CPU_INT08U   digits);*/


int main(void)
{
    INT8U  os_err;

    
    BSP_IntDisAll();                                            /* Disable all ints until we are ready to accept them.  */
    
    OSInit();                                                   /* Initialize "uC/OS-II, The Real-Time Kernel".         */

    os_err = OSTaskCreateExt((void (*)(void *)) App_TaskStart,  /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&App_TaskStartStk[APP_TASK_START_STK_SIZE - 1],
                             (INT8U           ) APP_TASK_START_PRIO,
                             (INT16U          ) APP_TASK_START_PRIO,
                             (OS_STK        * )&App_TaskStartStk[0],
                             (INT32U          ) APP_TASK_START_STK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

    #if (OS_TASK_NAME_EN > 0)
       OSTaskNameSet(APP_TASK_START_PRIO, (INT8U *)"Start Task", &os_err);
    #endif

    OSStart();                                                  /* Start multitasking (i.e. give control to uC/OS-II).  */

    return 0;
}
//----------------------------------------------------------------
static  void  App_TaskStart (void *p_arg)
{
    void  *msg;
    (void)p_arg;
    BSP_Init();                                                 /* Initialize BSP functions.                            */
    OS_CPU_SysTickInit();                                       /* Initialize the SysTick.                              */

#if (OS_TASK_STAT_EN > 0)
    OSStatInit();                                               /* Determine CPU capacity.                              */
#endif


    App_EventCreate();                                          /* Create application events.                           */
    App_TaskCreate();                                           /* Create application tasks.                            */

	//
		App_init();
    while (1) {     
        msg = OSMboxAccept(App_StartMbox);
				MainTaskProc(msg);
    }
}

/*
*********************************************************************************************************
*                                             App_EventCreate()
*
* Description : Create the application events.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TaskStart().
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_EventCreate (void)
{
#if (OS_EVENT_NAME_EN > 0)
    INT8U  os_err;
#endif

    App_RUNMbox = OSMboxCreate((void *)0);                   /* Create MBOX */
#if (OS_EVENT_NAME_EN > 0)
    OSEventNameSet(App_RUNMbox, "RUN Mbox", &os_err);
#endif

    App_UARTMbox = OSMboxCreate((void *)0);                   /* Create MBOX */
#if (OS_EVENT_NAME_EN > 0)
    OSEventNameSet(App_UARTMbox, "UART Mbox", &os_err);
#endif

    App_StartMbox = OSMboxCreate((void *)0);                   /* Create MBOX */
#if (OS_EVENT_NAME_EN > 0)
    OSEventNameSet(App_StartMbox, "Start Mbox", &os_err);
#endif
    
    //App_RunMbox = OSMboxCreate((void *)0);                   /* Create MBOX */
//#if (OS_EVENT_NAME_EN > 0)
//    OSEventNameSet(App_RunMbox, "Run Mbox", &os_err);
//#endif    
}

/*
*********************************************************************************************************
*                                            App_TaskCreate()
*
* Description : Create the application tasks.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : App_TaskStart().
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_TaskCreate (void)
{
    INT8U  os_err;
    os_err = OSTaskCreateExt((void (*)(void *)) App_TaskUart,
                             (void          * ) 0,
                             (OS_STK        * )&App_TaskUartStk[APP_TASK_UART_STK_SIZE - 1],
                             (INT8U           ) APP_TASK_UART_PRIO,
                             (INT16U          ) APP_TASK_UART_PRIO,
                             (OS_STK        * )&App_TaskUartStk[0],
                             (INT32U          ) APP_TASK_UART_STK_SIZE,
                             (void          * ) 0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

#if (OS_TASK_NAME_EN > 0)
    OSTaskNameSet(APP_TASK_UART_PRIO, "Uart", &os_err); 
#endif
//---------------------------------------     
    os_err = OSTaskCreateExt((void (*)(void *)) App_TaskSENSOR,
                             (void          * ) 0,
                             (OS_STK        * )&App_TaskSenSorStk[APP_TASK_SENSOR_STK_SIZE - 1],
                             (INT8U           ) APP_TASK_SENSOR_PRIO,
                             (INT16U          ) APP_TASK_SENSOR_PRIO,
                             (OS_STK        * )&App_TaskSenSorStk[0],
                             (INT32U          ) APP_TASK_SENSOR_STK_SIZE,
                             (void          * ) 0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

#if (OS_TASK_NAME_EN > 0)
    OSTaskNameSet(APP_TASK_SENSOR_PRIO, "SENSOR", &os_err);    
#endif   
//---------------------------------------   
    os_err = OSTaskCreateExt((void (*)(void *)) App_TaskRun,
                             (void          * ) 0,
                             (OS_STK        * )&App_TaskRunStk[APP_TASK_RUN_STK_SIZE - 1],
                             (INT8U           ) APP_TASK_RUN_PRIO,
                             (INT16U          ) APP_TASK_RUN_PRIO,
                             (OS_STK        * )&App_TaskRunStk[0],
                             (INT32U          ) APP_TASK_RUN_STK_SIZE,
                             (void          * ) 0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

#if (OS_TASK_NAME_EN > 0)
    OSTaskNameSet(APP_TASK_RUN_PRIO, "RUN", &os_err);    
#endif       
    
}
/*
*********************************************************************************************************
*                                              App_TaskKbd()
*
* Description : Monitor the state of the push buttons and passes messages to App_TaskUserIF()
*
* Argument(s) : p_arg       Argument passed to 'App_TaskKbd()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Note(s)     : none.
*********************************************************************************************************
*/
static  void  App_TaskUart (void *p_arg)
{
	INT32U  *msg;
	INT8U err;
    (void)p_arg;
    while (1) {     
        msg = (INT32U *)(OSMboxPend(App_UARTMbox, 0, &err));
		if(err==OS_ERR_NONE){
			if((INT32U)msg>0){
				if(msg[0]>>16)
					UartSendProc(msg[0]&0xff,(msg[0]>>8)&0xff);
				else
					UartRecvProc((msg[0]>>8)&0xff,(INT8U *)msg[1],msg[0]&0xff);

			}
		}
    }
}

//---------------------------------------------------------------
static  void  App_TaskSENSOR (void *p_arg)
{
     (void)p_arg;
			OSTimeDly(100);	
			while(1){
           	SensorProc();
					OSTimeDly(10);
			}
	
}

//---------------------------------------------------------------
static  void  App_TaskRun (void *p_arg)
{
	 void  *msg;
	/* INT8U err;
   (void)p_arg;
			while(1){msg = (INT32U *)(OSMboxPend(App_RUNMbox, 0, &err));
			if(err==OS_ERR_NONE)
          AppRunProc();
	} */
	   while(1) {    
			msg = OSMboxAccept(App_RUNMbox);
			AppRunProc(msg);
		 }
}

