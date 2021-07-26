/*
 * FreeRTOS V202104.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/**
 * This version of flash .c is for use on systems that have limited stack space
 * and no display facilities.  The complete version can be found in the
 * Demo/Common/Full directory.
 *
 * Three tasks are created, each of which flash an LED at a different rate.  The first
 * LED flashes every 200ms, the second every 400ms, the third every 600ms.
 *
 * The LED flash tasks provide instant visual feedback.  They show that the scheduler
 * is still operational.
 *
 */


#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_usart.h"
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_gpio.h"
/* Demo program include files. */
#include "partest.h"
#include "flash.h"
#include "portmacro.h"
#include "queue.h"
#include "stream_buffer.h"
#include "semphr.h"

#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE
#define ledNUMBER_OF_LEDS	( 3 )
#define ledFLASH_RATE_BASE	( ( TickType_t ) 333 )

//variable defination
char Read_str[20];
int i=0;

//Define queue length
static uint8_t msg_queue_len = 5;
static uint8_t msg_queue1_len = 10;
static uint8_t msg_queue2_len = 10;
//Queue handle
static QueueHandle_t msg_queue,msg_queue1, msg_queue2;

//function declaration
void usart_SendString(USART_TypeDef* USARTx, uint8_t * str);
void UART5_Configuration(uint32_t Baud);
void usart_SendChar(USART_TypeDef* USARTx, uint8_t ch);
void usart5_sendString(uint8_t *str);

/* Variable used by the created tasks to calculate the LED number to use, and
the rate at which they should flash the LED. */
static volatile UBaseType_t uxFlashTaskNumber = 0;

/* The task that is created three times. */
static portTASK_FUNCTION_PROTO( vLEDFlashTask, pvParameters );
// my function
static portTASK_FUNCTION_PROTO( vTaskMyLed_1, pvParameters );
static portTASK_FUNCTION_PROTO( vTaskMyLed_2, pvParameters );
static portTASK_FUNCTION_PROTO( vTaskMyButton, pvParameters );
static portTASK_FUNCTION_PROTO( newTask, pvParameters );

// stream buffer 
#define sbiSTREAM_BUFFER_LENGTH_BYTES		( ( size_t ) 100 )
#define sbiSTREAM_BUFFER_TRIGGER_LEVEL_10	( ( BaseType_t ) 10 )
/* The stream buffer that is used to send data from an interrupt to the task. */
static StreamBufferHandle_t xStreamBuffer = NULL;
/* The string that is sent from the interrupt to the task four bytes at a
time.  Must be multiple of 4 bytes long as the ISR sends 4 bytes at a time*/
static const char * pcStringToSend = "_____Hello FreeRTOS_____";

/*-----------------------------------------------------------*/
static TaskHandle_t xLed1_Task;
// mutex
static SemaphoreHandle_t xMutex;
void vStartLEDFlashTasks( UBaseType_t uxPriority )
{
        xTaskCreate( vTaskMyLed_1, "LED_1", ledSTACK_SIZE, NULL, tskIDLE_PRIORITY, &xLed1_Task );
        xTaskCreate( vTaskMyLed_2, "LED_2", ledSTACK_SIZE, NULL, tskIDLE_PRIORITY, ( TaskHandle_t * ) NULL );
        xTaskCreate( vTaskMyButton, "button", ledSTACK_SIZE, NULL, tskIDLE_PRIORITY, ( TaskHandle_t * ) NULL );
        xTaskCreate( newTask, "rxTask", ledSTACK_SIZE, NULL, tskIDLE_PRIORITY, ( TaskHandle_t * ) NULL );
        msg_queue = xQueueCreate(msg_queue_len, sizeof(int));
        msg_queue1 = xQueueCreate(msg_queue1_len, sizeof(char));
        msg_queue2 = xQueueCreate(msg_queue2_len, 10*sizeof(char));
        /* Create the stream buffer that sends data from the interrupt to the
	task, and create the task. */
	xStreamBuffer = xStreamBufferCreate( /* The buffer length in bytes. */
                                             sbiSTREAM_BUFFER_LENGTH_BYTES,
                                             /* The stream buffer's trigger level. */
                                             sbiSTREAM_BUFFER_TRIGGER_LEVEL_10 );
        //
        UART5_Configuration(4800);
        xMutex = xSemaphoreCreateMutex();
}

/*-----------------------------------------------------------*/
// Task to be created.
// This is the way this demo does it
// extermely bad for viewing, but provide portablility(I think)
static portTASK_FUNCTION( newTask, pvParameters ) // this will expend to vTaskCode()
{
  char tempChar1=0;
  static uint8_t test_array[10];
  static uint8_t index=0;
  for( ;; )
  {
    // recive queue
    if(xQueueReceive(msg_queue1,(void *) &tempChar1,0)==pdTRUE){
      usart_SendChar(UART5,tempChar1); // echo back
      test_array[index++] = tempChar1;
      if(index>=10)
      {
        index=0;
      }
      if(tempChar1==0x0d)
      {
        index=0;
        usart_SendChar(UART5,0x0a);
        xQueueSend(msg_queue2,(void *)test_array,10);
      }
    }
    vTaskDelay(100);
    // Task code goes here.
  }
}
/*-----------------------------------------------------------*/
// Task to be created.
// This is the way this demo does it
// extermely bad for viewing, but provide portablility(I think)
static portTASK_FUNCTION( vTaskMyLed_1, pvParameters ) // this will expend to vTaskCode()
{
  uint8_t toggle_state=0;
  uint8_t item;
  char tempChar;
  static uint8_t item_array[20];
  uint8_t index=0;
  for( ;; )
  {
    //uint32_t state = ulTaskNotifyTakeIndexed( 0,               /* Use the 0th notification */
     //                            pdTRUE,          /* Clear the notification value 
      //                                               before exiting. */
       //                          0 ); /* Block indefinitely. */
    
    if(xQueueReceive(msg_queue,(void *) &item,0)==pdTRUE){
      asm("NOP");
      item_array[index++] = item;
      if(toggle_state==1)
      {
        toggle_state=0;
      }
      else
      {
        toggle_state=1;
      }
    }
    // recive 2nd que
    //if(xQueueReceive(msg_queue1,(void *) &tempChar,0)==pdTRUE){
    //  asm("NOP");
   // }
    if(toggle_state == 1)
    {
      vParTestToggleLED( 3 );
    }
    else
    {
      vParTestToggleLED( 1 );
    }
    xSemaphoreTake( xMutex, portMAX_DELAY );
    usart5_sendString("vTaskMyLed_1: ------------------------------\r\n");
    xSemaphoreGive( xMutex );
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // Task code goes here.
  }
}
/*-----------------------------------------------------------*/
// Task to be created.
// This is the way this demo does it
// extermely bad for viewing, but provide portablility(I think)
static portTASK_FUNCTION( vTaskMyButton, pvParameters ) // this will expend to vTaskCode()
{
  uint8_t btn_state_now;
  uint8_t btn_state_prev;
  char testArray[10]={0};
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
        // for safty
       // if(xLed1_Task != NULL)
        //{
          //xTaskNotifyGiveIndexed( xLed1_Task, 0 );      
        //}
        if(xQueueSend(msg_queue,(void *)&num,10) != pdTRUE){
        //queue full
        }
        usart5_sendString("Ali Harmain\r\n");
        num++;
      }
    }
    // recive command
    if(xQueueReceive(msg_queue2,(void *) testArray,0)==pdTRUE){
      if(strcmp(testArray,"sannan\r")==0)
      {
        usart5_sendString("Hello sannan\r\n");
      }
      else if(strcmp(testArray,"change\r")==0)
      {
        xQueueSend(msg_queue,(void *)&num,10);
        num++;
      }
    }
    vTaskDelay(80);
    // Task code goes here.
  }
}
/*-----------------------------------------------------------*/
// Task to be created.
// This is the way this demo does it
// extermely bad for viewing, but provide portablility(I think)
static portTASK_FUNCTION( vTaskMyLed_2, pvParameters ) // this will expend to vTaskCode()
{
  static char cRxBuffer[ 20 ];
  static BaseType_t xNextByte = 0;
  size_t xReceivedBytes;
  const TickType_t xBlockTime = pdMS_TO_TICKS( 20 );
  for( ;; )
  {
    // recive one byte at a time
    xReceivedBytes = xStreamBufferReceive( /* The stream buffer data is being received from. */
                          xStreamBuffer,
                          /* Where to place received data. */
                          ( void * ) &( cRxBuffer[ xNextByte ] ),
                          /* The number of bytes to receive. */
                          2*sizeof( char ),
                          /* The time to wait for the next data if the buffer
                          is empty. */
                          xBlockTime );
    if( xReceivedBytes > 0 )
    {
      xNextByte+=2;
      if(xNextByte>=20)
      {
        xNextByte=0; // roll back
      }
    }
    vParTestToggleLED( 0 );
    xSemaphoreTake( xMutex, portMAX_DELAY );
    usart5_sendString("vTaskMyLed_2: ******************************\r\n");
    xSemaphoreGive( xMutex );
    vTaskDelay(rand()%500);
    // Task code goes here.
  }
}
/*-----------------------------------------------------------*/
static portTASK_FUNCTION( vLEDFlashTask, pvParameters )
{
TickType_t xFlashRate, xLastFlashTime;
UBaseType_t uxLED;

	/* The parameters are not used. */
	( void ) pvParameters;

	/* Calculate the LED and flash rate. */
	portENTER_CRITICAL();
	{
		/* See which of the eight LED's we should use. */
		uxLED = uxFlashTaskNumber;

		/* Update so the next task uses the next LED. */
		uxFlashTaskNumber++;
	}
	portEXIT_CRITICAL();

	xFlashRate = ledFLASH_RATE_BASE + ( ledFLASH_RATE_BASE * ( TickType_t ) uxLED );
	xFlashRate /= portTICK_PERIOD_MS;

	/* We will turn the LED on and off again in the delay period, so each
	delay is only half the total period. */
	xFlashRate /= ( TickType_t ) 2;

	/* We need to initialise xLastFlashTime prior to the first call to
	vTaskDelayUntil(). */
	xLastFlashTime = xTaskGetTickCount();

	for(;;)
	{
		/* Delay for half the flash period then turn the LED on. */
		vTaskDelayUntil( &xLastFlashTime, xFlashRate );
		vParTestToggleLED( uxLED );

		/* Delay for half the flash period then turn the LED off. */
		vTaskDelayUntil( &xLastFlashTime, xFlashRate );
		vParTestToggleLED( uxLED );
	}
} /*lint !e715 !e818 !e830 Function definition must be standard for task creation. */

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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;  //lowest priority
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
// set priority of 6
void UART5_IRQHandler(void)
{
  // stream buffer
  // used for indexing
  static size_t xNextByteToSend = 0;
  const BaseType_t xBytesToSend=1;
  //definations for QueuefromISR
  char cIn;
  BaseType_t xHigherPriorityTaskWoken;
  // We have not woken a task at the start of the ISR.
  xHigherPriorityTaskWoken = pdFALSE;
  if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
  {
    //code here
    //Read_str[i++]=USART_ReceiveData(UART5);
    cIn = USART_ReceiveData(UART5);
   // if(i==10){
   //   i=0;
   // }
    
    // Post the byte.
    xQueueSendFromISR( msg_queue1, &cIn, &xHigherPriorityTaskWoken );
    xStreamBufferSendFromISR( xStreamBuffer,
                              ( const void * ) ( pcStringToSend + xNextByteToSend ),
                              xBytesToSend,
                              NULL );
    xNextByteToSend += xBytesToSend;
    if( xNextByteToSend >= strlen( pcStringToSend ) )
    {
            xNextByteToSend = 0;
    }
    USART_ClearITPendingBit(UART5, USART_IT_RXNE);
    // context switch at the end
    // Now the buffer is empty we can switch context if necessary.
    if( xHigherPriorityTaskWoken )
    {
      // Actual macro used here is port specific.
      portYIELD_FROM_ISR (pdTRUE);
    }
  }
}