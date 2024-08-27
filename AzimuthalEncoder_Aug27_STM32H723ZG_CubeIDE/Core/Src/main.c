/**
  ******************************************************************************
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "lwip/udp.h"
#include <string.h>
#include <stdbool.h>

SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_tx;
DMA_HandleTypeDef hdma_spi4_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim23;

osThreadId defaultTaskHandle;
osThreadId ethernetTaskHandle;

uint16_t spiData[700];
uint16_t tempBuffer[601]; // 200 for value/value2, 200 for timer5val, 100 for 'f0f0' pattern, 1 for packet number

volatile uint16_t spiIndex = 0;
volatile uint32_t sampleNum = 0;

uint16_t txBuffer[3] = {0b1010101010101010, 0b1010101010101010, 0b1010101010111111};
uint16_t rxBuffer[3];

volatile uint16_t value;
volatile uint16_t value2;
volatile uint32_t timer23val;

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM23_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void startEthernetTask(void const * argument);

int main(void)
{
  // MPU Config
  MPU_Config();

  // Enable I-Cache
  SCB_EnableICache();

  // Enable D-Cache
  SCB_EnableDCache();

  // Reset of all peripherals, Initializes the Flash interface and SYSTICK
  HAL_Init();

  // Configure the system clock
  SystemClock_Config();

  // Initialize all configured peripherals
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_SPI4_Init();
  MX_TIM23_Init();
  MX_TIM1_Init();

  // Definition and creation of defaultTask
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  // Definition and creation of ethernetTask
  osThreadDef(ethernetTask, startEthernetTask, osPriorityHigh, 0, 256);
  ethernetTaskHandle = osThreadCreate(osThread(ethernetTask), NULL);

  // Start scheduler
  osKernelStart();

  // Never proceeds past here. Control taken by scheduler.

  /* Infinite loop */
  while (1)
  {
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Supply configuration update enable
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  // Configure the main internal regulator output voltage
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  // Initializes the RCC Oscillators according to the specified parameters in the RCC_OscInitTypeDef structure
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Initializes the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI4_Init(void)
{
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1375-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 275-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM23_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim23.Instance = TIM23;
  htim23.Init.Prescaler = 275-1;
  htim23.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim23.Init.Period = 4294967295;
  htim23.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim23.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim23) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim23, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim23, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DMA_Init(void)
{

  // DMA controller clock enable
  __HAL_RCC_DMA1_CLK_ENABLE();

  // DMA1_Stream0_IRQn interrupt configuration
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  // DMA1_Stream1_IRQn interrupt configuration
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  // Configure GPIO pin Output Level
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);

  // Configure GPIO pin PG12
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI4) {

		// For 16 bit value
		value = ( ((rxBuffer[0] & 0b0001000000000000) << 3) +
				  ((rxBuffer[0] & 0b0000010000000000) << 4) +
				  ((rxBuffer[0] & 0b0000000100000000) << 5)+
				  ((rxBuffer[0] & 0b0000000001000000) << 6)+
				  ((rxBuffer[0] & 0b0000000000010000) << 7)+
				  ((rxBuffer[0] & 0b0000000000000100) << 8)+
				  ((rxBuffer[0] & 0b0000000000000001) << 9)+
				  ((rxBuffer[1] & 0b0100000000000000) >> 6) +
				  ((rxBuffer[1] & 0b0001000000000000) >> 5) +
				  ((rxBuffer[1] & 0b0000010000000000) >> 4) +
				  ((rxBuffer[1] & 0b0000000100000000) >> 3) +
				  ((rxBuffer[1] & 0b0000000001000000) >> 2) +
				  ((rxBuffer[1] & 0b0000000000010000) >> 1) +
				  ((rxBuffer[1] & 0b0000000000000100)     ) +
				  ((rxBuffer[1] & 0b0000000000000001) << 1) +
				  ((rxBuffer[2] & 0b0100000000000000) >> 14));

		// Last 4 bits
		value2 = (((rxBuffer[2] & 0b0001000000000000) >> 9) +
				  ((rxBuffer[2] & 0b0000010000000000) >> 8) +
				  ((rxBuffer[2] & 0b0000000100000000) >> 7) +
				  ((rxBuffer[2] & 0b0000000001000000) >> 6));

		// Note: Optional to read the last bits. They should be 0s to ensure no short circuit

		spiData[spiIndex] = value; // first 16 bits
		spiData[spiIndex + 1] = value2; // last 4 bits
		spiData[spiIndex + 2] = (timer23val & 0xFFFF); // 32 bit clock tick
		spiData[spiIndex + 3] = ((timer23val >> 16) & 0xFFFF); // 32 bit clock tick
		spiData[spiIndex + 4] = 0xAB89; // 16 bit spacer
		spiData[spiIndex + 5] = 0xEFCD; // 16 bit spacer (two are necessary due to timer23val being 32 bit -- all 16 bit values are eventually covered)

		spiIndex = spiIndex + 6;

		if (spiIndex >= 700) {
			spiIndex = 0; // TODO find better solution than re-wrapping -- should stop taking data and alert
		}

		else if (spiIndex == 600) {
			spiData[spiIndex] = sampleNum;
			sampleNum++;
			spiIndex++;

			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(ethernetTaskHandle, &xHigherPriorityTaskWoken); // function will set xHigherPriorityTaskWoken to pdTRUE if the unblocked task (ethernetTaskHandle) has a higher priority than the currently running task. Also unblocks task
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // if xHigherPriorityTaskWoken is pdTURE, scheduler will switch to the ethernetTaskHandle task as soon as the ISR completes. Otherwise, currently running task will continue to run after ISR completes
		}
	}
}

void StartDefaultTask(void const * argument)
{
  /* Infinite loop */
  for (;;) {
	  osDelay(1);
  }
}

void startEthernetTask(void const * argument)
{
	MX_LWIP_Init();

	osDelay(100); // let LWIP be initialized

	// Bind the block to module's IP and port
	ip_addr_t myIPaddr;
	IP_ADDR4(&myIPaddr, 192, 168, 0, 123);

	ip_addr_t PC_IPADDR;
	IP_ADDR4(&PC_IPADDR, 192, 168, 0, 2);

	struct udp_pcb* my_udp = udp_new();
	udp_bind(my_udp, &myIPaddr, 8);
	udp_connect(my_udp, &PC_IPADDR, 55151);
	struct pbuf* udp_buffer = NULL;

	// Set PG12 to high to select 'transmit' on Arduino RS422 shield
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, 1);

    /* Start Timer 5 */
	HAL_TIM_Base_Start(&htim23);
	HAL_TIM_Base_Start(&htim1);

	// Start Timer 2 with 1ms interrupts
	HAL_TIM_Base_Start_IT(&htim2);

	for(;;)
	{
	  // Wait for the notification to send data
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	  // Copy samples from spiData to tempBuffer
	  memcpy(tempBuffer, spiData, sizeof(tempBuffer));


	  // Send the data over Ethernet
	  udp_buffer = pbuf_alloc(PBUF_TRANSPORT, sizeof(tempBuffer), PBUF_RAM);
	  if (udp_buffer != NULL)
	  {
		  memcpy(udp_buffer->payload, tempBuffer, sizeof(tempBuffer));
		  udp_send(my_udp, udp_buffer);
		  pbuf_free(udp_buffer);
	  }

	  // Shift the remaining samples up in the spiData buffer (pointer to dest, pointer to source, number of bytes)
	  memmove(spiData, &spiData[601], sizeof(spiData) - sizeof(tempBuffer));

	  // Update spiIndex to reflect the new starting position
	  spiIndex -= 601;

	}
}

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  // Disables the MPU
  HAL_MPU_Disable();

  // Initializes and configures the Region and the memory to be protected
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  // Initializes and configures the Region and the memory to be protected
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  // Initializes and configures the Region and the memory to be protected
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30000200;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // TIM6 is being used for SYSTICK since FreeRTOS is active
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }

  // TIM2 fires every 1ms and initializes SPI query to encoder. Can use HAL_SPI_TransmitReceive_DMA as well, but DMA must be re-initialized as it is not on circular mode.
  if (htim->Instance == TIM2) {
	  HAL_SPI_TransmitReceive_IT(&hspi4, (uint8_t *)txBuffer, (uint8_t *)rxBuffer, 3);
	  timer23val = __HAL_TIM_GET_COUNTER(&htim23);
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
