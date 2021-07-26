# FreeRTOS

## PortMacro

This is the function defined in portmacro.h.

```c
// The braces and semicolon are just of formating
{
	// A macro used to define prototype of a task function
	portTASK_FUNCTION_PROTO( vFunction, pvParameters );
	// A macro used to define the body of a task function
	portTASK_FUNCTION( vFunction, pvParameters );
    // This are port specific and call vPortEnterCritical, and vPortExitCritical/
    /* Use taskENTER_CRITICAL(), and taskEXIT_CRITICAL() instead form API for app writing. */
    portENTER_CRITICAL();
    portEXIT_CRITICAL();
    // Request context switch from ISR, the taskYIELD() dont have the ISR safe funtionc
    // so use this is ISR, will do the same thing
    portYIELD_FROM_ISR (pdTRUE);
}
```



## Task Creation

### xTaskCreate

Create a new task and add it to the list of tasks that are ready to run. 

> **Note**: configSUPPORT_DYNAMIC_ALLOCATION must be set to 1 in FreeRTOSConfig.h(or not be defined at all), or left undefined (in which case it will default to 1), for this RTOS API function to be available.

**Example usage:**

```c
/* configMINIMAL_STACK_SIZE is configured in FreeRTOSConfig.h, this is set to 130 in this example */
#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE'
/* see portmacro.h, this just expends to -> static void vTaskMyLed_1(pvParameters)*/
static portTASK_FUNCTION_PROTO( vTaskMyLed_1, pvParameters );
void main()
{
    /* Some stuff */
    // tskIDLE_PRIORITY = 0 // see task.h and is the lowest priority task
    // "LEDx" is the task name
    xTaskCreate( vTaskMyLed_1,
                 "LEDx",
                 ledSTACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY,
                 ( TaskHandle_t * ) NULL );
   /* som other stuff */
}
/* This just expends to -> static void vTaskMyLed_1(pvParameters) */
static portTASK_FUNCTION( vTaskMyLed_1, pvParameters ) // this will expend to vTaskCode()
{
    /* Task Initilization */
    for( ;; )
    {
        /* Task Code */
        vParTestToggleLED( 3 ); // toggle an led
        vTaskDelay(60); // wait 60 ms, blocked state
        // Task code goes here.
    }
    /* No work should be done here */
}
```

### Configuration parameter

```c
/* The maximum task priority ideal taks is '0' and max task priority is (configMAX_PRIORITIES-1), thus task priority are 0,1,2 and 3. This can be changed */
#define configMAX_PRIORITIES			( 5 )
```



## Task Control

### vTaskDelay

Delay a task for a given number of ticks. vTaskDelay() specifies a time at which the task wishes to unblock **relative to** the time at which vTaskDelay() is called.

The constant portTICK_PERIOD_MS can be used to calculate real time from the tick rate - with the resolution of one tick period.

The tick rate is 1ms for freeRTOS for, it can be configured using 'configTICK_RATE_HZ'. 100 Hz is normally a good choice. Higher tick rate provide higher resolution time, in expense of processing more tick interrupts.

> **Note**: INCLUDE_vTaskDelay must be defined as 1 for this function to be available.

See code in "xTaskCreate()" for an example usage.

### vTaskDelayUntil

Delay a task until a specified time. This function can be used by periodic tasks to ensure a constant execution frequency.

This function differs from vTaskDelay() in one important aspect: vTaskDelay() specifies a time at which the task wishes to unblock *relative* to the time at which vTaskDelay() is called, whereas vTaskDelayUntil() specifies an *absolute* time at which the task wishes to unblock.

> **Note**: INCLUDE_vTaskDelayUntil must be defined as 1 for this function to be available

**Example usage:**

```c
 // Perform an action every 10 ticks.
 void vTaskFunction( void * pvParameters )
 {
 TickType_t xLastWakeTime;
 const TickType_t xFrequency = 10;

     // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();

     for( ;; )
     {
         // Wait for the next cycle.
         vTaskDelayUntil( &xLastWakeTime, xFrequency );
         // Perform action here.
     }
 }
```

### Task Control Configuration

These need to be define in rtos configuration file to use taskDelay functions.

```c
// set to 1 to use vTaskDelayUntil()
#define INCLUDE_vTaskDelayUntil			1
// set to 1 to use vTaskDelay()
#define INCLUDE_vTaskDelay				1
```

## Queue creation

### xQueueCreate

xQueueCreate will create Queue. It will take two argument i.e. Queue length and size of object to be placed inside the queue. This function should be called in main function. Queue handle should be made, that will have the status of the xQueueCreate function.

The example code uses a queue to send data from a 'task' to a 2nd 'task'. The 2nd task will send a message when a button is pressed.

```c
//Define queue length
static uint8_t msg_queue_len = 5;

//Queue handle
static QueueHandle_t msg_queue;

//Queue create
// call this is a function, or main setup function, this is just an example
void setupFunction()
{
	msg_queue = xQueueCreate(msg_queue_len, sizeof(int));
}
//create task function
static portTASK_FUNCTION_PROTO( Queue_sendTask_button, pvParameters );
static portTASK_FUNCTION_PROTO( Queue_receiveTask, pvParameters );

//In main
void setupFunction2()
{
    xTaskCreate( Queue_sendTask,
    			"send",
    			configMINIMAL_STACK_SIZE,
    			NULL,
    			tskIDLE_PRIORITY,
    			( TaskHandle_t * ) NULL );
    xTaskCreate( Queue_receiveTask,
    			 "receive",
    			 configMINIMAL_STACK_SIZE,
    			 NULL,
    			 tskIDLE_PRIORITY,
    			 ( TaskHandle_t * ) NULL );
}
// this will expend to vTaskCode()
static Queue_receiveTask( Queue_receiveTask, pvParameters )
{
  uint8_t item;
  uint8_t index=0;
  for( ;; )
  {
    
    if(xQueueReceive(msg_queue,(void *) &item,0)==pdTRUE){
      asm("NOP");
      item_array[index++] = item;
    }
    vTaskDelay(100);
    // Task code goes here.
  }
}

static portTASK_FUNCTION( Queue_sendTask_button, pvParameters ) 
{
  uint8_t btn_state_now;
  uint8_t btn_state_prev;
  static uint8_t num=0;
  for( ;; )
  {
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==Bit_RESET)
    {
      btn_state_now=0;
    }
    else
    {
      btn_state_now=1;
    }
    if(btn_state_now != btn_state_prev)
    {
      btn_state_prev=btn_state_now;
      if(btn_state_now==0)
      {
       
        if(xQueueSend(msg_queue,(void *)&num,10) != pdTRUE){
        //queue full
        }
        num++;
      }
    }
    vTaskDelay(100);
    // Task code goes here.
  }
}
```



## UART Transmit and Receive 

We will configure UART same as in SPL(Standard peripheral library). Only one thing will change in the configuration that is the priority of the interrupt. Its priority should be lower configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.  

 The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! **(higher priorities are lower numeric values. )**

```c
//This is already defined in FreeRTOSConfig.h file
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5 
```

Thus the ISR should be of value higher or equal to 5. This means interrupts of 5,6,7..15 can use the FreeRTOS ISR safe function inside them, and ISR of 0,1,2,3 and 4 cannot use the FreeRTOS ISR safe functions.(ISR safe function are define in FreeRTOS docs)

The below example receives characters from uart5 and echo them back using a task, this demonstrate queues.

### UART Configuration

```c
// configure usart5 as tx and rx
void UART5_Configuration(uint32_t Baud)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	// Enable GPIOC CLOCK
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	//Enable GPIOD CLOCK
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//Enable UART5 CLOCK
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	// Configure the PD12 as a UART5 alternate pin.
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);

	GPIO_InitStructure.GPIO_OType					= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 					= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode 					= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin 					= GPIO_Pin_12;					//Configure only TX
	GPIO_InitStructure.GPIO_Speed 				= GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Configure the PD2 as a UART5 alternate pin.
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

	GPIO_InitStructure.GPIO_OType					= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 					= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode 					= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin 					= GPIO_Pin_2;					//Configure only TX
	GPIO_InitStructure.GPIO_Speed 				= GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	/* Enable the UART5 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    //low priority campare to 5
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 	   = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	USART_InitStructure.USART_BaudRate   			= Baud;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity  		    = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode				= USART_Mode_Rx | USART_Mode_Tx;

	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); // Enable USART1 Receive Interrupt
	USART_Init(UART5, &USART_InitStructure);
	USART_Cmd(UART5,  ENABLE);
}
// sending functions
void usart_SendString(USART_TypeDef* USARTx, uint8_t * str)
{
	while (*str != '\0')
	{
		usart_SendChar(USARTx,*str);
		str++;
	}
}
// usart_SendChar(UART5,'a');
void usart_SendChar(USART_TypeDef* USARTx, uint8_t ch)
{
	while (!(USARTx->SR & USART_SR_TXE))
		; // wait for TX to empty
	USART_SendData(USARTx, ch & 0xFF);
	while (!(USARTx->SR & USART_SR_TC))
		; // wait for Transmit complete
}
void usart5_sendString(uint8_t *str)
{
	usart_SendString(UART5,str);
}
```

### UART Receive Data

We will use ISR to receive the data and save it in Queue. we will first create the queue and specify its length.

```c
// this should be in scope of both sender and reciver
//Define queue length
static uint8_t msg_queue1_len = 10;

//Queue handle
static QueueHandle_t msg_queue1;
```



```c
//Queue Creation
// in main setup function
msg_queue1 = xQueueCreate(msg_queue1_len, sizeof(char));
```



```c
void UART5_IRQHandler(void)
{
  //definations for QueuefromISR
  char cIn;
  BaseType_t xHigherPriorityTaskWoken;
  // We have not woken a task at the start of the ISR.
  xHigherPriorityTaskWoken = pdFALSE;
  if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
  {
    cIn = USART_ReceiveData(UART5);
    // Post the byte.
    xQueueSendFromISR( msg_queue1, &cIn, &xHigherPriorityTaskWoken );
    // Now the buffer is empty we can switch context if necessary.
    if( xHigherPriorityTaskWoken )
    {
      // Actual macro used here is port specific.
      portYIELD_FROM_ISR (pdTRUE);
    }
    USART_ClearITPendingBit(UART5, USART_IT_RXNE);
  }
}
```

### UART Send Data

 We will make a task to receive the queue that is send from the interrupt. 

```c
//task function
static portTASK_FUNCTION_PROTO( newTask, pvParameters );

//task creation in main function
xTaskCreate( newTask, "rxTask", ledSTACK_SIZE, NULL, tskIDLE_PRIORITY, ( TaskHandle_t * ) NULL );
```

```c
static portTASK_FUNCTION( newTask, pvParameters ) // this will expend to vTaskCode()
{
    char tempChar1=0;
    static uint8_t item_array[20];
    for( ;; )
    {
        // receive queue
        if(xQueueReceive(msg_queue1,(void *) &tempChar1,0)==pdTRUE)
        {
            usart_SendChar(UART5,tempChar1);
        }
        vTaskDelay(100);   //reduce the delay to see the received data quickly.
        // Task code goes here.
    }
}
```



## RTOS Kernel Control

### taskYIELD

taskYIELD() is used to request a context switch to another task. However, if there are no other tasks at a higher or equal priority to the task that calls taskYIELD() then the RTOS scheduler will simply select the task that called taskYIELD() to run again. Used in cooperative scheduling.

To run the scheduler in cooperative mode set configUSE_PREEMPTION to 0, and for running it in preemptive mode set it to 1.

### Kernel Parameters

```c
/* This is used to select the sheduler mode, cooperative or preemptive. 1 means preemptive mode and 0 means cooperative mode */
#define configUSE_PREEMPTION                    1
/* seems to work with only preemptive mode, if not defined it is set to 1 */
#define configUSE_TIME_SLICING    1
/* The cpu clock that is driving the tick intrrupt, in case of stm32f405 this is the system clock */
#define configCPU_CLOCK_HZ				( SystemCoreClock )
/* This is the tick frequency 1000Hz means that the kernal is running a 1ms accuracy */
#define configTICK_RATE_HZ				( ( TickType_t ) 1000 )
```

### Stream Buffers file

The underline information is given in the message.

```c
/*
 * Stream buffers are used to send a continuous stream of data from one task or
 * interrupt to another.  Their implementation is light weight, making them
 * particularly suited for interrupt to task and core to core communication
 * scenarios.
 *
 * ***NOTE***:  Uniquely among FreeRTOS objects, the stream buffer
 * implementation (so also the message buffer implementation, as message buffers
 * are built on top of stream buffers) assumes there is only one task or
 * interrupt that will write to the buffer (the writer), and only one task or
 * interrupt that will read from the buffer (the reader).  It is safe for the
 * writer and reader to be different tasks or interrupts, but, unlike other
 * FreeRTOS objects, it is not safe to have multiple different writers or
 * multiple different readers.  If there are to be multiple different writers
 * then the application writer must place each call to a writing API function
 * (such as xStreamBufferSend()) inside a critical section and set the send
 * block time to 0.  Likewise, if there are to be multiple different readers
 * then the application writer must place each call to a reading API function
 * (such as xStreamBufferReceive()) inside a critical section section and set the
 * receive block time to 0.
 *
 * Use xStreamBufferSend() to write to a stream buffer from a task.  Use
 * xStreamBufferSendFromISR() to write to a stream buffer from an interrupt
 * service routine (ISR).
 */
```

### Example

```c
// stream buffer 
// size of buffer
#define sbiSTREAM_BUFFER_LENGTH_BYTES		( ( size_t ) 100 )
// the trigger level is the number of bytes that should be in the buffer
// for it to unblock a task, used when using portMAX_DELAY with
// buffer recive function
#define sbiSTREAM_BUFFER_TRIGGER_LEVEL_10	( ( BaseType_t ) 10 )
/* The stream buffer that is used to send data from an interrupt to the task. */
static StreamBufferHandle_t xStreamBuffer = NULL;
/* The string that is sent from the interrupt to the task four bytes at a
time.  Must be multiple of 4 bytes long as the ISR sends 4 bytes at a time*/
static const char * pcStringToSend = "_____Hello FreeRTOS_____";
setup()
{
    /* Create the stream buffer that sends data from the interrupt to the
	task, and create the task. */
    // task created recivingTask();
	xStreamBuffer = xStreamBufferCreate( /* The buffer length in bytes. */
                                             sbiSTREAM_BUFFER_LENGTH_BYTES,
                                             /* The stream buffer's trigger level. */
                                             sbiSTREAM_BUFFER_TRIGGER_LEVEL_10 );
}
recivingTask()
{
    static char cRxBuffer[ 20 ];
    static BaseType_t xNextByte = 0;
    size_t xReceivedBytes;
    const TickType_t xBlockTime = pdMS_TO_TICKS( 20 );
    for(;;)
    {
        xReceivedBytes = xStreamBufferReceive( 
            /* The stream buffer data is being received from. */
            xStreamBuffer,
            /* Where to place received data. */
            ( void * ) &( cRxBuffer[ xNextByte ] ),
            /* The number of bytes to receive. */
            sizeof( char ),
            /* The time to wait for the next data if the buffer
                          is empty. */
            xBlockTime );
        if( xReceivedBytes > 0 )
        {
            xNextByte++;
            if(xNextByte>=20)
            {
                xNextByte=0; // roll back
            }
        }
    }
}
ISR()
{
    static size_t xNextByteToSend = 0;
  	const BaseType_t xBytesToSend=4;
    BaseType_t xHigherPriorityTaskWoken;
  	// We have not woken a task at the start of the ISR.
  	xHigherPriorityTaskWoken = pdFALSE;
    
    xStreamBufferSendFromISR( xStreamBuffer,
                              ( const void * ) ( pcStringToSend + xNextByteToSend ),
                              xBytesToSend,
                              &xHigherPriorityTaskWoken );
    xNextByteToSend += xBytesToSend;
    if( xNextByteToSend >= strlen( pcStringToSend ) )
    {
            xNextByteToSend = 0;
    }
    // yield
    if( xHigherPriorityTaskWoken )
    {
      // Actual macro used here is port specific.
      portYIELD_FROM_ISR (pdTRUE);
    }
}
```



## Customization

### Configuration

FreeRTOS is customized using a configuration file called FreeRTOSConfig.h. Blow is the content of the file with a short description each parameter, this is not the complete list. For a complete list refer to FreeRTOS website.

```c
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Here is a good place to include header files that are required across
your application. */
#include "something.h"
/* This should be kept as provided in demo app, this is in words not bytes(1 word is 4 bytes), thus it is 130*4 = 520 bytes */
#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 130 )
/* Memory allocation related definitions. */
/* These are set to their default values(we will not define them), see FreeRtos.h */

/* Optional functions */
/* Define features that are to be used, like taskDelay */
/* A header file that defines trace macro can be included here. */

#endif /* FREERTOS_CONFIG_H */
```

### Memory Management

The FreeRtos version 10 provide five heap management schemes. Only one scheme file should be used at a time(personal preference). Following below:

- heap_1 - the very simplest, does not permit memory to be freed.
- heap_2 - permits memory to be freed, but does not coalescence adjacent free blocks.
- heap_3 - simply wraps the standard malloc() and free() for thread safety.
- heap_4 - coalescences adjacent free blocks to avoid fragmentation. Includes absolute address placement option.
- heap_5 - as per heap_4, with the ability to span the heap across multiple non-adjacent memory areas.

Notes:

- heap_1 is less useful since FreeRTOS added support for static allocation.
- heap_2 is now considered legacy as the newer heap_4 implementation is preferred.

### Stream Buffers vs Message Buffers vs Queues

Here is a reply from 'Richard Barry' creator of FreeRtos.

> In case you are not aware, there are pages on the FreeRTOS.org site that describe each in some detail.
>
> Short reply as using my cellphone.
>
> Stream buffers carry messages that don’t necessary have a beginning and end, hence stream. Thing of a series of bytes arriving on a serial port.
>
> Message buffers hold discrete messages that have a fixed size, but each message does not have to be the same size (variable sized messages).
>
> The above two are light weight implementations that are fast but have usage restrictions documented on the website.
>
> Queues are much more sophisticated objects with multiple senders and receivers that are held in priority lists. Each message sent to a queue must be the same size, the size being specified when the queue is created. The website also documents techniques
> to use queues for variable size messages to, but only indirectly.

### RTOS Task Notifications

portMAX_DELAY means wait forever. notification used to sync two task, as binary semaphores. 

function used.

xTaskNotifyGiveIndexed( xLed1_Task, 0 );

ulTaskNotifyTakeIndexed( 0,pdTRUE,portMAX_DELAY );

```
static TaskHandle_t xLed1_Task; // must be global
void main()
{
xTaskCreate( vTaskMyLed_1, "LED_1", ledSTACK_SIZE, NULL, tskIDLE_PRIORITY, &xLed1_Task );
}
static portTASK_FUNCTION( vTaskMyLed_1, pvParameters ) // this will expend to vTaskCode()
{ 
  for( ;; )
  {
    ulTaskNotifyTakeIndexed( 0,               /* Use the 0th notification */
                                 pdTRUE,          /* Clear the notification value 
                                                     before exiting. */
                                 portMAX_DELAY ); /* Block indefinitely. */
    vParTestToggleLED( 3 );
    vTaskDelay(60);
    // Task code goes here.
  }
}
static portTASK_FUNCTION( vTaskMyButton, pvParameters )
{
  for( ;; )
  {
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==Bit_RESET)
    {
      if(xLed1_Task != NULL)
      {
        xTaskNotifyGiveIndexed( xLed1_Task, 0 );
      }
    }
    vTaskDelay(100);
    // Task code goes here.
  }
}
```
