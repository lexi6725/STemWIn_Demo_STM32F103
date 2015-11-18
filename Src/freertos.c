/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "GUI.h"
#include "WM.h"
#include "GUIDEMO.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId AppStartHandle;
osThreadId LedFlashHandle;
osThreadId CommRFHandle;
osThreadId DMPHandle;
osThreadId MagMeterHandle;
osThreadId BackgroundHandle;
osMutexId I2C_MutexHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void LedFlashTask(void const * argument);
void CommRFTask(void const * argument);
void DMP_Task(void const * argument);
void MagMeterTask(void const * argument);
void Background_Task(void const * argument);

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */
	   
	/* USER CODE END Init */

	/* Create the mutex(es) */
	/* definition and creation of I2C_Mutex */
	osMutexDef(I2C_Mutex);
	I2C_MutexHandle = osMutexCreate(osMutex(I2C_Mutex));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of Background */
	osThreadDef(Background, Background_Task, osPriorityHigh, 0, 512);
	BackgroundHandle = osThreadCreate(osThread(Background), NULL);

	/* definition and creation of AppStart */
	osThreadDef(AppStart, StartDefaultTask, osPriorityNormal, 0, 3014);
	AppStartHandle = osThreadCreate(osThread(AppStart), NULL);

	/* definition and creation of LedFlash */
	osThreadDef(LedFlash, LedFlashTask, osPriorityNormal, 0, 64);
	LedFlashHandle = osThreadCreate(osThread(LedFlash), NULL);

	/* definition and creation of CommRF */
	osThreadDef(CommRF, CommRFTask, osPriorityHigh, 0, 128);
	CommRFHandle = osThreadCreate(osThread(CommRF), NULL);

	/* definition and creation of DMP */
	osThreadDef(DMP, DMP_Task, osPriorityNormal, 0, 128);
	DMPHandle = osThreadCreate(osThread(DMP), NULL);

	/* definition and creation of MagMeter */
	osThreadDef(MagMeter, MagMeterTask, osPriorityIdle, 0, 64);
	MagMeterHandle = osThreadCreate(osThread(MagMeter), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void Background_Task(void const * argument)
{
	static uint32_t ticks = 0;

	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(10);
	}
	/* USER CODE END StartDefaultTask */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  //MX_FATFS_Init();
  static int count = 0;

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    GUIDEMO_Main();
    /*if (count < 15)
    {
	    GUI_DispString("Hello STemWin!");
	    GUI_DispNextLine();
	    osDelay(500);
	    GUI_DispString("abcdefg");
	    GUI_DispNextLine();
	    osDelay(500);
	    count++;
    }
    else
    {
    	GUI_Clear();
    	count = 0;
    }*/
  }
  /* USER CODE END StartDefaultTask */
}

/* LedFlashTask function */
void LedFlashTask(void const * argument)
{
  /* USER CODE BEGIN LedFlashTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END LedFlashTask */
}

/* CommRFTask function */
void CommRFTask(void const * argument)
{
  /* USER CODE BEGIN CommRFTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CommRFTask */
}

/* DMP_Task function */
void DMP_Task(void const * argument)
{
  /* USER CODE BEGIN DMP_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DMP_Task */
}

/* MagMeterTask function */
void MagMeterTask(void const * argument)
{
  /* USER CODE BEGIN MagMeterTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MagMeterTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
