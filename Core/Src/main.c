/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "main.h"
#include "cmsis_os.h"


UART_HandleTypeDef huart2; //uart 2 handle
UART_HandleTypeDef huart3; //uart 3 handle

/* error counters */
volatile uint32_t errors_uart2 = 0;
volatile uint32_t errors_uart3 = 0;


typedef uint8_t data_t; /// data type for uart (if data==9bit need a uint16)

/*--------------- Definitions for RTOS objects--------------------------------*/

/* queues */
#define QUEUE_MAX_SIZE 		128 /// queue capacity

osMessageQueueId_t qRxUart3;	/// writes in uart3 irg, reads in uart 3 task */
osMessageQueueId_t qRxUart2;	/// writes in uart2 irg, reads in uart 2 task */


osSemaphoreId_t semUart3tx; /// releases in uart3 irg, takes in task uart2 */
osSemaphoreId_t semUart2tx; /// releases in uart2 irg, takes in task uart3 */


osThreadId_t uart3TaskHandle;
const osThreadAttr_t uart3Task_attributes = {
	.name = "uart3tsk",
	.stack_size = 128*4,
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t uart2TaskHandle;
const osThreadAttr_t uart2Task_attributes = {
	.name = "uart2tsk",
	.stack_size = 128*4,
	.priority = (osPriority_t) osPriorityNormal,
};


/*  init prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* tasks prototypes */
void uartTask(void *argument);
void StartDefaultTask(void *param);




/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick*/
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();

	osKernelInitialize();

	//Init tx semaphores
	semUart2tx = osSemaphoreNew(1,1,NULL);
	semUart3tx = osSemaphoreNew(1,1,NULL);

	//init queues capacity=QUEUE_MAX_SIZE, size of element = 1or2 byte;
	qRxUart2 = osMessageQueueNew(QUEUE_MAX_SIZE, sizeof(data_t), NULL);
	qRxUart3 = osMessageQueueNew(QUEUE_MAX_SIZE, sizeof(data_t), NULL);

	//init uart tasks
	uart3TaskHandle = osThreadNew(uartTask, (void*) 3, &uart3Task_attributes);
	uart2TaskHandle = osThreadNew(uartTask, (void*) 2, &uart2Task_attributes);

	/* Start scheduler */
	osKernelStart();

	while (1)
	{
	  ;
	}

}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = (sizeof(data_t)==2)? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief USART3 Initialization Function
  */
static void MX_USART3_UART_Init(void)
{
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = (sizeof(data_t)==2)? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief GPIO Initialization Function for LED
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}


/**
  * @brief Will be called in the end of UART ISR, after receiving 1 byte
  * @param huart - uart handle
  *
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* TODO: it seems like hardcode. May be other way exists to get last byte
	data in UART with HAL uart using. I need to get uart data. I get it
	by decrementing data ptr in huart instance; This code should be
	safe, because HAL_UART_RxCpltCallback() calls only in ISR  */

	data_t data = *(data_t*)(huart->pRxBuffPtr-sizeof(data));
	osMessageQueueId_t qHandle;

	/* find queue handle by uart instance */
	if (huart->Instance == USART3) {
		qHandle = qRxUart3;
	}
	else if  (huart->Instance == USART2) {
		qHandle = qRxUart2;
	}


	/* if error while receive occur put it to queue instead data */
	if(huart->ErrorCode) {
		data = huart->ErrorCode;
	}

	/* cmsis os wraps about freertos and check if this is IRQ or not */
	osMessageQueuePut(qHandle, &data,0,0);

	/* debug led blink */
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	/* prepare to receiving new byte of data, HAL_UART_Receive_IT enables uart
	interrupts, and waits for 1 byte (wait for call HAL_UART_RxCpltCallback()
	in next ISR) */
	HAL_UART_Receive_IT(huart, &data, 1);
}


/**
  * @brief Will be called in the end of UART ISR, after packet transmitting
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	osSemaphoreId_t 	sem;

	/* uart3 worries about RX3 queue and TX2 resource */
	if (huart->Instance == USART3) {
		sem = semUart3tx;
	}
	else if  (huart->Instance == USART2) {
		sem = semUart2tx;
	}
	else {
		Error_Handler();
	}


	osSemaphoreRelease(sem);
	/* In this callback an old transmission was completed yet,
	so we can start a new transmission in task. Semaphore notices
	task about free resource */
}

/**
  * @brief Will be called at the end in uart ISR, if any errors occurred ;
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	static uint32_t errors_UART2=0;
	static uint32_t errors_UART3=0;

	if (huart->Instance == USART3)
		errors_UART3++;
	else
		errors_UART2++;
}


/**
  * @brief  Task handles uart routine.
  *
  * We will create 2 tasks with this function. When queue qRxUart3, that fill in
  * UART3 ISR, will get any data - task unlocks and put data from transmitter
  * UART2. It is correct for task uart3tsk. For uart2tsk we wait for qRxUart2
  * and transmit data by UART3
  *
  * @param  u8_uartNum: number of uart (2 or 3)
  * @retval None
  */
void uartTask(void *u8_uartNum)
{
	data_t data_arr[QUEUE_MAX_SIZE];
	uint32_t size;

	UART_HandleTypeDef 	*pHuartRX;
	UART_HandleTypeDef 	*pHuartTX;
	osMessageQueueId_t 	queueRX;
	osSemaphoreId_t		sem;

	//uart2 worry about RX2 and TX3
	if ((uint8_t)u8_uartNum == 2) {
		pHuartRX = &huart2;
		queueRX = qRxUart2;

		sem = semUart3tx;
		pHuartTX = &huart3;
	}
	else if ((uint8_t)u8_uartNum == 3) {
		//uart3 worry about RX3 and TX2
		pHuartRX = &huart3;
		queueRX = qRxUart3;

		sem = semUart2tx;
		pHuartTX = &huart2;
	}
	else {
		Error_Handler();
	}

	/* Initializes receive data. It enables IRQ. When a byte received, ISR
	callback will puts it to queue */
	HAL_UART_Receive_IT(pHuartRX, &data_arr[0], 1);
	for(;;)
	{
		/* The thread will blocked while queue is empty */
		osMessageQueueGet(queueRX, &data_arr[0], 0, portMAX_DELAY);

		/* the thread unblocked and puts a byte to transfer */

		/* calculate size - how much we need keeping in mind
		previous call of osMessageQueueGet(). We already get 1 byte */
		size = 1;
		size += osMessageQueueGetCount(queueRX);

		for (int i=1; i<size; i++) {
			osMessageQueueGet(queueRX, &data_arr[i], 0, portMAX_DELAY);
		}

		/* init transmission */
		HAL_UART_Transmit_IT(pHuartTX, data_arr, size);

		/* wait for transmission have done. Semaphore
		will release in HAL_UART_TxCpltCallback() */
		osSemaphoreAcquire(sem, portMAX_DELAY);

	}
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  ;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
