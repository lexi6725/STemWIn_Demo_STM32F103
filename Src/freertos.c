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
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "led.h"
#include "mpu6050.h"
#include "i2c.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId AppStartHandle;
osThreadId LedFlashHandle;
osThreadId CommRFHandle;
osThreadId DMPHandle;
osThreadId MagMeterHandle;
osMutexId I2C_MutexHandle;

/* USER CODE BEGIN Variables */

// EventGroup
EventGroupHandle_t xRFEventGruop = NULL;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void LedFlashTask(void const * argument);
void CommRFTask(void const * argument);
void DMP_Task(void const * argument);
void MagMeterTask(void const * argument);

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
  
  /* USER CODE BEGIN RTOS_EventGroups */
  /* add eventgroups, add new ones, ... */
  xRFEventGruop = xEventGroupCreate();
  /* USER CODE END RTOS_EventGroups */

  /* Create the thread(s) */
  /* definition and creation of AppStart */
  osThreadDef(AppStart, StartDefaultTask, osPriorityNormal, 0, 128);
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
  osThreadDef(MagMeter, MagMeterTask, osPriorityNormal, 0, 64);
  MagMeterHandle = osThreadCreate(osThread(MagMeter), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* LedFlashTask function */
void LedFlashTask(void const * argument)
{
	/* USER CODE BEGIN LedFlashTask */
	TickType_t xLastTime;
	
	LED_Init(LED1);

	xLastTime = xTaskGetTickCount();
	
	/* Infinite loop */
	for(;;)
	{
		LED_Toggle(LED1);
		osDelayUntil(&xLastTime, 100);
	}
	/* USER CODE END LedFlashTask */
}

/* CommRFTask function */
void CommRFTask(void const * argument)
{
	/* USER CODE BEGIN CommRFTask */

	//rf_mode_rx();
	
	/* Infinite loop */
	for(;;)
	{
		osDelay(1000);
	}
	/* USER CODE END CommRFTask */
}

/* DMP_Task function */
void DMP_Task(void const * argument)
{
  /* USER CODE BEGIN DMP_Task */
extern struct hal_s hal;
    unsigned long sensor_timestamp;

    LED_Init(LED2);
    mpu6050_init();
    
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
		#if 1
		if (hal.new_gyro && hal.dmp_on) 
		{
	        short gyro[3], accel[3], sensors;
	        unsigned char more;
	        long quat[4];
			LED_Toggle(LED2);
	        /* This function gets new data from the FIFO when the DMP is in
	         * use. The FIFO can contain any combination of gyro, accel,
	         * quaternion, and gesture data. The sensors parameter tells the
	         * caller which data fields were actually populated with new data.
	         * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
	         * the FIFO isn't being filled with accel data.
	         * The driver parses the gesture data to determine if a gesture
	         * event has occurred; on an event, the application will be notified
	         * via a callback (assuming that a callback function was properly
	         * registered). The more parameter is non-zero if there are
	         * leftover packets in the FIFO.
	         */
	        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
	            &more);
	        if (!more)
	            hal.new_gyro = 0;
	        /* Gyro and accel data are written to the FIFO by the DMP in chip
	         * frame and hardware units. This behavior is convenient because it
	         * keeps the gyro and accel outputs of dmp_read_fifo and
	         * mpu_read_fifo consistent.
	         */
	        if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
	            send_packet(PACKET_TYPE_GYRO, gyro);
	        if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
	            send_packet(PACKET_TYPE_ACCEL, accel);
	        /* Unlike gyro and accel, quaternions are written to the FIFO in
	         * the body frame, q30. The orientation is set by the scalar passed
	         * to dmp_set_orientation during initialization.
	         */
	        if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
	            send_packet(PACKET_TYPE_QUAT, quat);
	    } 
	    else if (hal.new_gyro) 
	    {
	        short gyro[3], accel[3];
	        unsigned char sensors, more;
	        /* This function gets new data from the FIFO. The FIFO can contain
	         * gyro, accel, both, or neither. The sensors parameter tells the
	         * caller which data fields were actually populated with new data.
	         * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
	         * being filled with accel data. The more parameter is non-zero if
	         * there are leftover packets in the FIFO.
	         */
	        mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
	        if (!more)
	            hal.new_gyro = 0;
	        if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
	            send_packet(PACKET_TYPE_GYRO, gyro);
	        if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
	            send_packet(PACKET_TYPE_ACCEL, accel);
	    }
	    #endif
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
