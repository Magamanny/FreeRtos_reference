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

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_gpio.h"
/* Demo program include files. */
#include "partest.h"
#include "flash.h"

#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE
#define ledNUMBER_OF_LEDS	( 3 )
#define ledFLASH_RATE_BASE	( ( TickType_t ) 333 )

/* Variable used by the created tasks to calculate the LED number to use, and
the rate at which they should flash the LED. */
static volatile UBaseType_t uxFlashTaskNumber = 0;

/* The task that is created three times. */
static portTASK_FUNCTION_PROTO( vLEDFlashTask, pvParameters );
// my function
static portTASK_FUNCTION_PROTO( vTaskMyLed_1, pvParameters );
static portTASK_FUNCTION_PROTO( vTaskMyLed_2, pvParameters );
static portTASK_FUNCTION_PROTO( vTaskMyButton, pvParameters );
/*-----------------------------------------------------------*/
static TaskHandle_t xLed1_Task;
void vStartLEDFlashTasks( UBaseType_t uxPriority )
{
        xTaskCreate( vTaskMyLed_1, "LED_1", ledSTACK_SIZE, NULL, tskIDLE_PRIORITY, &xLed1_Task );
        xTaskCreate( vTaskMyLed_2, "LED_2", ledSTACK_SIZE, NULL, tskIDLE_PRIORITY, ( TaskHandle_t * ) NULL );
        xTaskCreate( vTaskMyButton, "button", ledSTACK_SIZE, NULL, tskIDLE_PRIORITY, ( TaskHandle_t * ) NULL );
}
/*-----------------------------------------------------------*/
// Task to be created.
// This is the way this demo does it
// extermely bad for viewing, but provide portablility(I think)
static portTASK_FUNCTION( vTaskMyLed_1, pvParameters ) // this will expend to vTaskCode()
{
  uint8_t toggle_state=0;
  for( ;; )
  {
    uint32_t state = ulTaskNotifyTakeIndexed( 0,               /* Use the 0th notification */
                                 pdTRUE,          /* Clear the notification value 
                                                     before exiting. */
                                 0 ); /* Block indefinitely. */
    if(state==1)
    {
      if(toggle_state==1)
      {
        toggle_state=0;
      }
      else
      {
        toggle_state=1;
      }
    }
    if(toggle_state == 1)
    {
      vParTestToggleLED( 3 );
    }
    vTaskDelay(60);
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
        if(xLed1_Task != NULL)
        {
          xTaskNotifyGiveIndexed( xLed1_Task, 0 );
        }
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
static portTASK_FUNCTION( vTaskMyLed_2, pvParameters ) // this will expend to vTaskCode()
{
  for( ;; )
  {
    vParTestToggleLED( 0 );
    vTaskDelay(80);
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

