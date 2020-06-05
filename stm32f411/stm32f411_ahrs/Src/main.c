/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "timers.h"
/* USER CODE BEGIN Includes */
#include "stm32f411e_discovery.h"
#include "stm32f411e_discovery_accelerometer.h"
#include "stm32f411e_discovery_gyroscope.h"
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osThreadId imuTaskHandle;
TimerHandle_t timer_os;
void timer_callback( TimerHandle_t xTimer );
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void EXTILine0_Config(void);
void UART_Init(void);
void StartImuTask(void const * argument);
void StartDefaultTask(void const * argument);
void vButtonTask(void const * argument);
void vTaskPrint(void const * argument);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static xSemaphoreHandle xButtonSemaphore;
/* USER CODE END 0 */

UART_HandleTypeDef huart2;
xSemaphoreHandle mutex = 0;
float q_ahrs[4], q_ahrs_2[4];

float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0f;
	}
	if (v >= 1.0f)
	{
		return M_PI / 2;
	}
	if (v <= -1.0f)
	{
		return -M_PI / 2;
	}
	return asin(v);
}
float invSqrt(float x)
{
	volatile float halfx = 0.5f * x;
	volatile float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	UART_Init();

	printf("Hello world!");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
	EXTILine0_Config();
  /* USER CODE END 2 */
  /* USER CODE BEGIN 5 */
	q_ahrs[0] = q_ahrs_2[0] = 1.0f;
	if(BSP_ACCELERO_Init() != HAL_OK)
  {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    /* Initialization Error */
    Error_Handler(); 
  }
	
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
		/* Ensure the semaphore is created before it gets used. */
	xButtonSemaphore = xSemaphoreCreateCounting( 10, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	timer_os = xTimerCreate("SW Timer", 1000, pdTRUE, (void *) 0, timer_callback);
	 //start Timer
   xTimerStart(timer_os, 0 );
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityRealtime, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	 
	osThreadDef(ImuTask, StartImuTask, osPriorityRealtime, 0, 128);
  imuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);
	/*
	interrupt handler task
	*/
	osThreadDef(buttonTask, vButtonTask, osPriorityRealtime, 0, 128);
  imuTaskHandle = osThreadCreate(osThread(buttonTask), NULL);
	
		osThreadDef(TaskPrint, vTaskPrint, osPriorityRealtime, 0, 128);
  imuTaskHandle = osThreadCreate(osThread(TaskPrint), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}
void timer_callback( TimerHandle_t xTimer )
{
		if( xSemaphoreTake(mutex, (TickType_t)0xFFFFFFFF) == pdTRUE) //ktra xem co the vao chua
		{
			printf("This is timer call back\n\r");
	    xSemaphoreGive( mutex ); //unlock mutex
		}
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0)
  {
		BaseType_t sHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR( xButtonSemaphore, &sHigherPriorityTaskWoken );
		if( sHigherPriorityTaskWoken == pdTRUE ) 
		{ 
		/* Giving the semaphore unblocked a task, and the priority of the 
		unblocked task is higher than the currentlyrunning task - force 
		a context switch to ensure that the interrupt returns directly to 
		the unblocked (higher priority) task. 
		NOTE: The actual macro to use to force a context switch from an 
		ISR is dependent on the port. This is the correct macro for the 
		Open Watcom DOS port. Other ports may require different syntax. 
		Refer to the examples provided for the portbeing used to determine 
		the syntax required. */ 
		portYIELD_FROM_ISR(sHigherPriorityTaskWoken); 
		}
  }
}

/* USER CODE BEGIN 4 */
static void EXTILine0_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    osDelay(100);
  }
  /* USER CODE END 5 */ 
}
TickType_t xLastExecutionTime;
void vTaskPrint(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		
		if( xSemaphoreTake(mutex, (TickType_t)0xFFFFFFFF) == pdTRUE) //ktra xem co the vao chua
		{
	    //printf("1234567890 Day la resource\r\n");
			printf("This is print task\n\r");
			xLastExecutionTime = xTaskGetTickCount(); 
			printf("TickCount: %d \n\r", xLastExecutionTime);
	    xSemaphoreGive( mutex ); //unlock mutex
		}
    osDelay(500);
  }
  /* USER CODE END 5 */ 
}

static int16_t accel_buffer[3];
static float gyro_buffer[3], gyro_rps[3];
#define GYRO_DPS 500.0f
#define GYRO_SCALE GYRO_DPS/32767.0f

volatile double halfT ,elapsedT;
float rpy[3], rpy2[3];
float tMagValue[3]={0, 0, 0};
#define MAG_SCALE 0.080e-3f
void StartImuTask(void const * argument)
{
  if(BSP_GYRO_Init() != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
	BSP_MAGNETO_Init();
  /* Infinite loop */
  for(;;)
  {
		BSP_ACCELERO_GetXYZ(accel_buffer);
		BSP_GYRO_GetXYZ(gyro_buffer);
		BSP_MAGNETO_GETXYZ(tMagValue);

		halfT = 0.004f;
		FreeIMU_AHRSupdate(q_ahrs, gyro_buffer[0] * M_PI / 180.0f,gyro_buffer[1] * M_PI / 180.0f,gyro_buffer[2] * M_PI / 180.0f, 
		accel_buffer[0],accel_buffer[1], accel_buffer[2]);
		IMU_getRollPitchYaw(rpy, q_ahrs);
		IMU_AHRSupdate(q_ahrs_2, gyro_buffer[0] * M_PI / 180.0f,gyro_buffer[1] * M_PI / 180.0f,gyro_buffer[2] * M_PI / 180.0f, 
		accel_buffer[0],accel_buffer[1], accel_buffer[2], tMagValue[0]*MAG_SCALE, tMagValue[1]*MAG_SCALE, tMagValue[2]*MAG_SCALE);
		IMU_getRollPitchYaw(rpy2, q_ahrs_2);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    osDelay(4);
		
  }
  /* USER CODE END 5 */ 
}
#define butDEBOUNCE_DELAY	( 200 / portTICK_RATE_MS )
uint16_t toggle_cnt;
void vButtonTask(void const * argument)
{
	xSemaphoreTake( xButtonSemaphore, portMAX_DELAY);
	
	for( ;; )
	{
		/* Block on the semaphore to wait for an interrupt event.  The semaphore
		is 'given' from vButtonISRHandler() below.  Using portMAX_DELAY as the
		block time will cause the task to block indefinitely provided
		INCLUDE_vTaskSuspend is set to 1 in FreeRTOSConfig.h. */
		xSemaphoreTake( xButtonSemaphore, portMAX_DELAY);
		printf("Button pressed detected, Semaphore Take!\n\r");
		/* The button must have been pushed for this line to be executed.
		Simply toggle the LED. */
		//butLED1 = !butLED1;
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		toggle_cnt++;
		/* Wait a short time then clear any pending button pushes as a crude
		method of debouncing the switch.  xSemaphoreTake() uses a block time of
		zero this time so it returns immediately rather than waiting for the
		interrupt to occur. */
	//	vTaskDelay( butDEBOUNCE_DELAY );
	//	xSemaphoreTake( xButtonSemaphore, 0 );
	}
}

void UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
/*
Calculate qa0...qa3
*/
volatile float integralFBx, integralFBy, integralFBz;

#define twoKpDef  (5.0f ) // 2 * proportional gain
#define twoKiDef  (0.2f) // 2 * integral gain

void FreeIMU_AHRSupdate(float* q_update, volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az)
{
	volatile float norm;
	//  float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz;
	volatile float ex, ey, ez;
	volatile float temp0, temp1, temp2, temp3;

	// ???????õõ???????
	volatile float q0q0 = q_update[0] * q_update[0];
	volatile float q0q1 = q_update[0] * q_update[1];
	volatile float q0q2 = q_update[0] * q_update[2];
	volatile float q0q3 = q_update[0] * q_update[3];
	volatile float q1q1 = q_update[1] * q_update[1];
	volatile float q1q2 = q_update[1] * q_update[2];
	volatile float q1q3 = q_update[1] * q_update[3];
	volatile float q2q2 = q_update[2] * q_update[2];
	volatile float q2q3 = q_update[2] * q_update[3];
	volatile float q3q3 = q_update[3] * q_update[3];

	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	// estimated direction of gravity and flux (v and w)
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay * vz - az * vy) ;
	ey = (az * vx - ax * vz) ;
	ez = (ax * vy - ay * vx) ;

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{

		integralFBx +=  ex * twoKiDef * halfT;
		integralFBy +=  ey * twoKiDef * halfT;
		integralFBz +=  ez * twoKiDef * halfT;

		gx = gx + twoKpDef * ex + integralFBx;
		gy = gy + twoKpDef * ey + integralFBy;
		gz = gz + twoKpDef * ez + integralFBz;

	}
	// integrate quaternion rate and normalise
	temp0 = q_update[0] + (double)(-q_update[1] * gx - q_update[2] * gy - q_update[3] * gz) * halfT;
	temp1 = q_update[1] + (double)(q_update[0] * gx + q_update[2] * gz - q_update[3] * gy) * halfT;
	temp2 = q_update[2] + (double)(q_update[0] * gy - q_update[1] * gz + q_update[3] * gx) * halfT;
	temp3 = q_update[3] + (double)(q_update[0] * gz + q_update[1] * gy - q_update[2] * gx) * halfT;

	// normalise quaternion
	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	q_update[0] = temp0 * norm;
	q_update[1] = temp1 * norm;
	q_update[2] = temp2 * norm;
	q_update[3] = temp3 * norm;
}

void IMU_getRollPitchYaw(float *angles, float *qa)
{
	/*
	 Quaternion To Euler function 
	*/
	angles[0] = (atan2(2.0f * (qa[0] * qa[1] + qa[2] * qa[3]),
				      1 - 2.0f * (qa[1] * qa[1] + qa[2] * qa[2]))) * 180 / M_PI;
	//angles[2]-=0.8f;
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	
	angles[1] = -safe_asin(2.0f * (qa[0] * qa[2] - qa[3] * qa[1])) * 180 / M_PI;
	angles[2] = -atan2(2 * qa[1] * qa[2] + 2 * qa[0] * qa[3], -2*qa[2]*qa[2] - 2*qa[3]*qa[3] + 1) * 180 / M_PI; // yaw
}

#define Kp 3.0f
#define Ki 0.03f
//float gyro_mahony[3];
volatile float exInt, eyInt, ezInt;  
void IMU_AHRSupdate(float* q_update, volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az, volatile float mx, volatile float my, volatile float mz)
{
	volatile  float norm;
	volatile float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz, wx, wy, wz;
	volatile float ex, ey, ez;
	volatile float temp0, temp1, temp2, temp3;
	volatile float temp;

	float q0q0 = q_update[0] * q_update[0];
	float q0q1 = q_update[0] * q_update[1];
	float q0q2 = q_update[0] * q_update[2];
	float q0q3 = q_update[0] * q_update[3];
	float q1q1 = q_update[1] * q_update[1];
	float q1q2 = q_update[1] * q_update[2];
	float q1q3 = q_update[1] * q_update[3];
	float q2q2 = q_update[2] * q_update[2];
	float q2q3 = q_update[2] * q_update[3];
	float q3q3 = q_update[3] * q_update[3];

	temp = sqrt(ax * ax + ay * ay + az * az);
	temp = (temp / 16384.0f) * 9.8f;   //M/S^2
//	acc_vector = acc_vector +   //??????????????20hz
//		     (halfT * 2.0f / (7.9577e-3f + halfT * 2.0f)) * (temp - acc_vector);
	
	// Normalise accelerometer measurement
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	//?ü???????roll??pitch
	//	temp = ax * invSqrt((ay * ay + az * az));
	//	ACC_Pitch = atan(temp)* 57.3;
	//
	//	temp = ay * invSqrt((ax * ax + az * az));
	//	ACC_Roll = atan(temp)* 57.3;
// Normalise magnetometer measurement
	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;
	
	// Reference direction of Earth's magnetic field
	// compute reference direction of flux

	hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
	hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
	hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
	/*

	*/
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;

	// estimated direction of gravity and flux (v and w)

	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	/*

	*/
	wx = 2 * bx * (0.5f - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
	wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
	wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5f - q1q1 - q2q2);

	// error is sum of cross product between reference direction of fields and direction measured by sensors

	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	/*

	*/
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;

		// adjusted gyroscope measurements

		gx = gx + (Kp * ex + exInt);
		gy = gy + (Kp * ey + eyInt);
		gz = gz + (Kp * ez + ezInt);
//    gyro_mahony[0] = gx*RAD_TO_DEG;
//		gyro_mahony[1] = gy*RAD_TO_DEG;
//		gyro_mahony[2] = gz*RAD_TO_DEG;
	}

	// integrate quaternion rate and normalise
	// ???????????
	temp0 = q_update[0] + (-q_update[1] * gx - q_update[2] * gy - q_update[3] * gz) * halfT;
	temp1 = q_update[1] + (q_update[0] * gx + q_update[2] * gz - q_update[3] * gy) * halfT;
	temp2 = q_update[2] + (q_update[0] * gy - q_update[1] * gz + q_update[3] * gx) * halfT;
	temp3 = q_update[3] + (q_update[0] * gz + q_update[1] * gy - q_update[2] * gx) * halfT;

	// normalise quaternion
	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	q_update[0] = temp0 * norm;
	q_update[1] = temp1 * norm;
	q_update[2] = temp2 * norm;
	q_update[3] = temp3 * norm;
}

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
