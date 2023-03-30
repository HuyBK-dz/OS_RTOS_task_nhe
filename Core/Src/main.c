/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CMD_OPT_NOT_USE                         0x00
#define CMD_OPT_EXISTCMD_MASK                   0x01
#define CMD_LENGTH_MAX							50 // bytes

/*! @brief Field type */
#define CMD_TYPE_GET                            0x00
#define CMD_TYPE_RES                            0x01
#define CMD_TYPE_SET                            0x02

/*! @brief Start of frame */
#define FRAME_SOF                               0xB1

/*! @brief Check xor init */
#define CXOR_INIT_VAL                           0xFF

#define CMD_ID_TRACKING_PERFORMANCE			    0x90

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
TaskHandle_t task1_handle = NULL;
TaskHandle_t task2_handle = NULL;
TaskHandle_t task3_handle = NULL;
TaskHandle_t task4_handle = NULL;
TaskHandle_t task5_handle = NULL;
TaskHandle_t task6_handle = NULL;
TaskHandle_t task7_handle = NULL;

uint8_t task1Delete_fg = 0;
uint8_t task2Delete_fg = 0;
uint8_t task3Delete_fg = 0;
uint8_t task4Delete_fg = 0;
uint8_t task5Delete_fg = 0;

uint32_t countTotal = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void task1_handler(void* parameters);
static void task2_handler(void* parameters);
static void task3_handler(void* parameters);
static void task4_handler(void* parameters);
static void task5_handler(void* parameters);
//static void task6_handler(void* parameters);
//static void task7_handler(void* parameters);
void Serial_SendPacket(uint8_t byOption,uint8_t byCmdId,uint8_t byType,uint8_t* pPayload,uint8_t byLengthPayload);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t day_fibonaci(uint32_t i)
{
   if(i == 0)
   {
      return 0;
   }
   if(i == 1)
   {
      return 1;
   }
   return day_fibonaci(i-1) + day_fibonaci(i-2);
}

uint32_t day_fibonaci2(uint32_t i)
{
   if(i == 0)
   {
      return 0;
   }
   if(i == 1)
   {
      return 1;
   }
   return day_fibonaci2(i-1) + day_fibonaci2(i-2);
}

uint32_t day_fibonaci3(uint32_t i)
{
   if(i == 0)
   {
      return 0;
   }
   if(i == 1)
   {
      return 1;
   }
   return day_fibonaci3(i-1) + day_fibonaci3(i-2);
}

uint32_t day_fibonaci4(uint32_t i)
{
   if(i == 0)
   {
      return 0;
   }
   if(i == 1)
   {
      return 1;
   }
   return day_fibonaci4(i-1) + day_fibonaci4(i-2);
}

uint32_t day_fibonaci5(uint32_t i)
{
   if(i == 0)
   {
      return 0;
   }
   if(i == 1)
   {
      return 1;
   }
   return day_fibonaci5(i-1) + day_fibonaci5(i-2);
}

size_t get_used_ram_size(void)
{
	uint32_t task1Stack = uxTaskGetStackHighWaterMark(task1_handle);
	uint32_t task2Stack = uxTaskGetStackHighWaterMark(task2_handle);
	uint32_t task3Stack = uxTaskGetStackHighWaterMark(task3_handle);
	uint32_t task4Stack = uxTaskGetStackHighWaterMark(task4_handle);
	uint32_t task5Stack = uxTaskGetStackHighWaterMark(task5_handle);

	uint32_t remainStack = (5 * configMINIMAL_STACK_SIZE) - (task1Stack + task2Stack + task3Stack + task4Stack + task5Stack); // + task6Stack + task7Stack);

	/* Calculate the total RAM used by the application */
	size_t xFreeHeapSize = xPortGetFreeHeapSize();
	size_t xRamUsed = configTOTAL_HEAP_SIZE - xFreeHeapSize + remainStack;

    return xRamUsed;
}

void vTimerCallback( TimerHandle_t xTimer )
{
	uint8_t byPayload[12] = {0};
	uint32_t ramUsed = get_used_ram_size();
	uint32_t tickTime = HAL_GetTick();

	// In log ra App sử dụng UART interface
	// Kiểu dữ liệu của các tham số countTotal, ramUsed, tickTime là 32 bit sắp xếp dữ liệu theo dạng Big Endiance
//	byPayload[0] = (countTotal >> 24) & 0xFF;
//	byPayload[1] = (countTotal >> 16) & 0xFF;
//	byPayload[2] = (countTotal >> 8) & 0xFF;
//	byPayload[3] = (countTotal & 0xFF);
//	byPayload[4] = (ramUsed >> 24) & 0xFF;
//	byPayload[5] = (ramUsed >> 16) & 0xFF;
//	byPayload[6] = (ramUsed >> 8) & 0xFF;
//	byPayload[7] = (ramUsed & 0xFF);
//	byPayload[8] = (tickTime >> 24) & 0xFF;
//	byPayload[9] = (tickTime >> 16) & 0xFF;
//	byPayload[10] = (tickTime >> 8) & 0xFF;
//	byPayload[11] = (tickTime & 0xFF);
//	Serial_SendPacket(CMD_OPT_NOT_USE, CMD_ID_TRACKING_PERFORMANCE, CMD_TYPE_RES, byPayload, sizeof(byPayload));

	// In log ra màn hình Console
	printf("c: %ld, r: %ld, t: %ld\n", countTotal, ramUsed, tickTime);

	// Waiting until bit flag delete to set 1
	if ((task1Delete_fg == 1) && (task2Delete_fg == 1) && (task3Delete_fg == 1) && (task4Delete_fg == 1) && (task5Delete_fg == 1))  {
		BaseType_t status;

		// Đặt lại cờ báo hiệu về chưa xóa task
		task1Delete_fg = 0;
		task2Delete_fg = 0;
		task3Delete_fg = 0;
		task4Delete_fg = 0;
		task5Delete_fg = 0;

		// Tạo lại các task khi đã xóa xong
		status = xTaskCreate(task1_handler, "Task_1", configMINIMAL_STACK_SIZE, NULL, 2, &task1_handle);

		// Kiểm tra trạng thái của task có được tạo thành công trên heap hay không?
		configASSERT(status == pdPASS);

		status = xTaskCreate(task2_handler, "Task_2", configMINIMAL_STACK_SIZE, NULL, 2, &task2_handle);

		configASSERT(status == pdPASS);

		status = xTaskCreate(task3_handler, "Task_3", configMINIMAL_STACK_SIZE, NULL, 2, &task3_handle);

		configASSERT(status == pdPASS);

		status = xTaskCreate(task4_handler, "Task_4", configMINIMAL_STACK_SIZE, NULL, 2, &task4_handle);

		configASSERT(status == pdPASS);

		status = xTaskCreate(task5_handler, "Task_5", configMINIMAL_STACK_SIZE, NULL, 2, &task5_handle);

		configASSERT(status == pdPASS);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	BaseType_t status;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	// Khởi tạo 5 task bằng lệnh create để đưa data của task vào heap
	status = xTaskCreate(task1_handler, "Task_1", configMINIMAL_STACK_SIZE, NULL, 2, &task1_handle);

	configASSERT(status == pdPASS);

	status = xTaskCreate(task2_handler, "Task_2", configMINIMAL_STACK_SIZE, NULL, 2, &task2_handle);

	configASSERT(status == pdPASS);

	status = xTaskCreate(task3_handler, "Task_3", configMINIMAL_STACK_SIZE, NULL, 2, &task3_handle);

	configASSERT(status == pdPASS);

	status = xTaskCreate(task4_handler, "Task_4", configMINIMAL_STACK_SIZE, NULL, 2, &task4_handle);

	configASSERT(status == pdPASS);

	status = xTaskCreate(task5_handler, "Task_5", configMINIMAL_STACK_SIZE, NULL, 2, &task5_handle);

	configASSERT(status == pdPASS);

//	status = xTaskCreate(task6_handler, "Task_6", 200, NULL, 2, &task6_handle);
//
//	configASSERT(status == pdPASS);
//
//	status = xTaskCreate(task7_handler, "Task_7", 200, NULL, 2, &task7_handle);
//
//	configASSERT(status == pdPASS);

	// Tạo ra timer 10ms để in log ra màn hình Console và sửa thành 15ms khi đưa log lên App
	TimerHandle_t xTimer = xTimerCreate( "Timer", pdMS_TO_TICKS( 10 ), pdTRUE, 0, vTimerCallback );
	if( xTimer == NULL ) {}
	xTimerStart( xTimer, 0 );

	// In ra giá trị RAM khi đã khởi các task nhưng các task chưa chạy
	printf("Init c: %ld, r: %ld, t: %ld\n", countTotal, (uint32_t)get_used_ram_size(), (uint32_t)HAL_GetTick());

	// start the freeRTOS scheduler
	vTaskStartScheduler();

	// if the control comes here, then the launch of the scheduler has failed
	// due to insufficient memory in heap

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
	Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */
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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
static void task1_handler(void* parameters)
{
	while(1)
	{
		uint32_t j = 1;
		for (uint32_t i = 1; i  < 5; i++) {
			j *= i;
		}
		vTaskDelay(pdMS_TO_TICKS( 10UL ));
		++countTotal;
		taskYIELD(); // Dùng để chuyển các task thực hiện

		// Xóa task khi đã thực hiện xong dãy fibonanci
//		vTaskDelete(task1_handle, &task1Delete_fg);
	}
}

static void task2_handler(void* parameters)
{
	while(1)
	{
		uint32_t j = 1;
		for (uint32_t i = 1; i  < 5; i++) {
			j *= i;
		}
		vTaskDelay(pdMS_TO_TICKS( 10UL ));
		++countTotal;
		taskYIELD();

//		vTaskDelete(task2_handle, &task2Delete_fg);
	}
}

static void task3_handler(void* parameters)
{
	while(1)
	{
		uint32_t j = 1;
		for (uint32_t i = 1; i  < 5; i++) {
			j *= i;
		}
		vTaskDelay(pdMS_TO_TICKS( 10UL ));
		++countTotal;
		taskYIELD();

//		vTaskDelete(task3_handle, &task3Delete_fg);
	}
}

static void task4_handler(void* parameters)
{
	while(1)
	{
		uint32_t j = 1;
		for (uint32_t i = 1; i  < 5; i++) {
			j *= i;
		}
		vTaskDelay(pdMS_TO_TICKS( 10UL ));
		++countTotal;
		taskYIELD();

//		vTaskDelete(task4_handle, &task4Delete_fg);
	}
}

static void task5_handler(void* parameters)
{
	while(1)
	{
		uint32_t j = 1;
		for (uint32_t i = 1; i  < 5; i++) {
			j *= i;
		}
		vTaskDelay(pdMS_TO_TICKS( 10UL ));
		++countTotal;
		taskYIELD();

//		vTaskDelete(task5_handle, &task5Delete_fg);
	}
}

//static void task6_handler(void* parameters)
//{
//	while(1)
//	{
//		for (uint32_t i = 0; i < 20; i++)
//		{
//		   day_fibonaci(i);
//		}
//
//		++countTotal;
//		taskYIELD();
//	}
//}
//
//static void task7_handler(void* parameters)
//{
//	while(1)
//	{
//		for (uint32_t i = 0; i < 20; i++)
//		{
//		   day_fibonaci(i);
//		}
//
//		++countTotal;
//		taskYIELD();
//	}
//}

/**
 * @func   CalculateCheckXOR
 * @brief  Calculate value XOR
 * @param  pbyTxBuffer:
           bySizeFrame: Size of frame
 * @retval check_XOR
 */
static uint8_t CalculateCheckXOR(uint8_t *pbyTxBuffer,uint8_t bySizeFrame)
{
    uint8_t byCXOR = CXOR_INIT_VAL;

    for (uint8_t byCntSize = 0; byCntSize < bySizeFrame; byCntSize++)
    {
        byCXOR ^= *pbyTxBuffer;
        pbyTxBuffer++;
    }
    return byCXOR;
}

/**
 * @func   Serial_SendPacket
 * @brief  Process transmit message uart
 * @param  byOption: Option
 * @param  byCmdId: Identify
 * @param  byType: Type
 * @param  pPayload: Payload
 * @param  byLengthPayload: Length payload
 * @retval None
 */
void
Serial_SendPacket(
    uint8_t byOption,
    uint8_t byCmdId,
    uint8_t byType,
    uint8_t* pPayload,
    uint8_t byLengthPayload
) {
    static uint8_t bySeq = 0;
    uint8_t byOffset = 0;
    uint8_t byBufferTx[CMD_LENGTH_MAX];
    uint8_t checkxor;

    byBufferTx[byOffset++] = FRAME_SOF;
    byBufferTx[byOffset++] = byLengthPayload + 5; /* Include: opt + id + type + seq + cxor */
    byBufferTx[byOffset++] = byOption;
    byBufferTx[byOffset++] = byCmdId;
    byBufferTx[byOffset++] = byType;

    for (uint8_t i = 0; i < byLengthPayload; i++) {
        byBufferTx[byOffset++] = pPayload[i];
    }

    byBufferTx[byOffset++] = bySeq++;
    checkxor = CalculateCheckXOR(&byBufferTx[2], byOffset - 2);
    byBufferTx[byOffset++] = checkxor;

    /* Send frame to Host via UART */
    HAL_UART_Transmit(&huart2, byBufferTx, byOffset, HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM5) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
