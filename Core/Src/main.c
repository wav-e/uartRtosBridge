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


UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;


typedef uint8_t data_t;

/*--------------- Definitions for RTOS objects--------------------------------*/
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

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

/* tasks prototypes */
void uartTask(void *argument);
void StartDefaultTask(void *param);

/* queues */
osMessageQueueId_t qRxUart3;
osMessageQueueId_t qRxUart2;
#define QUEUE_MAX_SIZE 10

osSemaphoreId_t semUart3;
osSemaphoreId_t semUart2;




/*----------------------- init prototypes ------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  osKernelInitialize();

  //init queues capacity=QUEUE_MAX_SIZE, size of element = 1or2 byte;
  semUart2 = osSemaphoreNew(1,1,NULL);
  semUart3 = osSemaphoreNew(1,1,NULL);
  qRxUart2 = osMessageQueueNew(QUEUE_MAX_SIZE, sizeof(data_t), NULL);
  qRxUart3 = osMessageQueueNew(QUEUE_MAX_SIZE, sizeof(data_t), NULL);

  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  uart3TaskHandle = osThreadNew(uartTask, (void*) 3, &uart3Task_attributes);
  uart2TaskHandle = osThreadNew(uartTask, (void*) 2, &uart2Task_attributes);


  /* Start scheduler */
  osKernelStart();


  while (1) {
	  ;
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
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
  * @param None
  * @retval None
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
  * @param None
  * @retval None
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
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


/** It will be called in the end of UART ISR, after receiving 1 byte */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	/* TODO: it seems like hardcode. May be other way exists to get
	 * last byte data in UART. I need to get uart data. I get it
	 * by decrementing data ptr in huart instance; This code should be
	 * safe, because HAL_UART_RxCpltCallback() calls only in ISR  */

	//TODO: 9 byte transfer support
	data_t data = *(huart->pRxBuffPtr-1);
	osMessageQueueId_t qHandle;

	if (huart->Instance == USART3) {
		qHandle = qRxUart3;
	}
	else if  (huart->Instance == USART2) {
		qHandle = qRxUart2;
	}
	else {
		Error_Handler();
	}


	osMessageQueuePut(qHandle, &data,0,0);

	//debug led blink
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	/* prepare to receiving new byte of data, HAL_UART_Receive_IT enables uart
	interrupts, and waits for 1 byte (wait for call HAL_UART_RxCpltCallback()
	in next ISR) */
	HAL_UART_Receive_IT(huart, &data, 1);
}

/** It will be called in the end of UART ISR, after transmitting 1 byte */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	osMessageQueueId_t 	qHandle;
	osSemaphoreId_t 	sem;

	// TODO: optimize it!!!
	static data_t data[QUEUE_MAX_SIZE]; //copy data from queue to this array for TX

	uint32_t size;

	if (huart->Instance == USART3) {
		qHandle = qRxUart2;
		sem = semUart3;
	}
	else if  (huart->Instance == USART2) {
		qHandle = qRxUart3;
		sem = semUart2;
	}
	else {
		Error_Handler();
	}

//	size = osMessageQueueGetCount(qHandle);
//
//	for (int i=0; i<size; i++) {
//		osMessageQueuePut(qHandle, &data[i],0,0);
//	}
//
//
//	/* In this callback an old transmission was
//	 * completed yet, so we can start a new
//	 * transmission. HAL_UART_Transmit_IT() checks
//	 * for size and just will return if size==0  */
//	HAL_UART_Transmit_IT(huart, &data, size);
//	if (huart->gState == HAL_UART_STATE_READY)
		osSemaphoreRelease(sem);
}


void StartDefaultTask(void *param) {
	uint8_t msg3[] = "UART 3 sent it\n";
	uint8_t msg2[] = "UART 2 sent it\n";


	for(;;){
		HAL_UART_Transmit_IT(&huart3, msg3, sizeof(msg3)-1);
		HAL_UART_Transmit_IT(&huart2, msg2, sizeof(msg2)-1);
		osDelay(500);
	}
}




/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
void uartTask(void *u8_uartNum)
{
	uint8_t msg[] = "uart3Task is running\n";
	data_t data;
	data_t data_arr[QUEUE_MAX_SIZE];
	uint32_t size;

	UART_HandleTypeDef 	*pHuartRX;
	UART_HandleTypeDef 	*pHuartTX;
	osMessageQueueId_t 	queueRX;
	osSemaphoreId_t		sem;
	//osMessageQueueId_t qHandleTX;

	if ((uint8_t)u8_uartNum == 2) {
		sem = semUart2;
		pHuartRX = &huart2;
		queueRX = qRxUart2;

		pHuartTX = &huart3;
	}
	else if ((uint8_t)u8_uartNum == 3) {
		sem = semUart3;
		pHuartRX = &huart3;
		queueRX = qRxUart3;

		pHuartTX = &huart2;
	}
	else {
		Error_Handler();
	}

	/* Initializes receive data. It enables IRQ. When a byte received, ISR
	callback will puts it to queue, and calls HAL_UART_Receive_IT() again */
	HAL_UART_Receive_IT(pHuartRX, &data, 1);
	for(;;)
	{
		/* The thread will blocked while queue is empty */
		osMessageQueueGet(queueRX, &data_arr[0], 0, portMAX_DELAY);

		/* the thread unblocked and puts a byte to transfer */
		// TODO: we need a semaphore if data transfers from ISR
		//osSemaphoreAcquire(sem, portMAX_DELAY);
		size = 1;
		size += osMessageQueueGetCount(queueRX);
		for (int i=1; i<size; i++) {
			osMessageQueueGet(queueRX, &data_arr[i],0,0);
		}



		HAL_UART_Transmit_IT(pHuartTX, data_arr, size);

		/* ISR will continue transfer */
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
