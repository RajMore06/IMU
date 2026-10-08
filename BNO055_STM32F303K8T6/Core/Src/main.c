/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "bno055.h"
#include "calib_storage.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char debug[64];
float quat[4];
float quat_filtered[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float heading_raw;
float heading_filtered = -1.0f;
float head;
static float headingOffset = 0.0f;
static bool headingZeroed = false;

uint8_t TxData[8];
uint8_t RxData[8];
uint16_t can_Head = 0;
uint32_t TxMailbox;
uint32_t last_tick = 0;          /* for fixed-rate loop */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

#define LOOP_PERIOD_MS       5U        /* 10 ms = 100 Hz */
#define QUAT_SLERP_ALPHA     0.15f      /* faster at 100 Hz — was 0.08 at 10 Hz */
#define HEADING_LPF_ALPHA    0.18f      /* faster at 100 Hz — was 0.10 at 10 Hz */
#define HEADING_JUMP_THRESH  60.0f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void canTransmit(uint16_t val);
void canInit_recieveMultiID(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_CAN_Start(&hcan);
  canInit_recieveMultiID();
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x691;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  bno055_Reset();
  bno055_Scan();
  bno055_Post();
  bno055_Init();

  uint8_t offsets[CALIB_OFFSET_COUNT];

  if (calib_storage_load(offsets))
  {
    //serialWrite("Calibration data found in Flash — restoring offsets.\n");
    bno055_Offset_Write(offsets, CALIB_OFFSET_COUNT, BNO055_OPERATION_MODE_NDOF);
    HAL_Delay(650);
    ///serialWrite("Offsets restored — sensor ready.\n");
  }
  else
  {
    //serialWrite("No calibration data in Flash — starting full calibration.\n");
    //serialWrite("Follow the physical procedure:\n");
    //serialWrite("  1. Set on flat surface — do not touch (gyro)\n");
    //serialWrite("  2. Rotate through 6 distinct faces, hold each 2-3s (acc)\n");
    //serialWrite("  3. Figure-8 motion away from metal objects (mag)\n");

    bno055_SetOprMode(BNO055_OPERATION_MODE_NDOF);
    HAL_Delay(650);

    /*
     * Poll until sys=3 gyr=3 acc=3 mag=3 (calib == 0xFF).
     * 300 000 ms = 5 minutes — enough for even a slow magnetometer.
     * Increase if needed.
     */
    uint8_t calib = 0;
    bno055_Calibration_Status(&calib, 60000);

    if (calib == 0xFF)
    {
      bno055_Offset_Read(offsets, CALIB_OFFSET_COUNT, BNO055_OPERATION_MODE_NDOF);

      HAL_StatusTypeDef flash_status = calib_storage_save(offsets);

      if (flash_status == HAL_OK)
      {
        //serialWrite("Offsets saved to Flash.\n");
        //serialWrite("Future resets will skip calibration automatically.\n");
      }
      else
      {
        /*
         * Flash write failed — sensor is calibrated for this session
         * but offsets will be lost on next reset.
         */
        //snprintf(debug, sizeof(debug),
          //       "Flash write fail (HAL error %d). "
            //     "Offsets lost on reset.\n", (int)flash_status);
        //serialWrite(debug);
      }
    }
    else
    {
      //serialWrite("WARNING: Calibration timed out — partial offsets NOT saved.\n");
      //serialWrite("Heading may be inaccurate this session.\n");
    }
  }

  //serialWrite("Waiting for fusion engine...\n");
    {
      uint32_t t0 = HAL_GetTick();
      do {
        bno055_ReadQuatData(quat);
        HAL_Delay(10);
      } while (!bno055_QuatIsValid(quat) && (HAL_GetTick() - t0) < 1000U);

      if (bno055_QuatIsValid(quat))
      {
        /* Seed the SLERP accumulator and heading filter */
        quat_filtered[0] = quat[0];
        quat_filtered[1] = quat[1];
        quat_filtered[2] = quat[2];
        quat_filtered[3] = quat[3];
        heading_filtered  = bno055_QuatToHeading(quat_filtered);
       // serialWrite("Fusion engine ready.\n");
      }
      else
      {
        //serialWrite("WARNING: Fusion engine not ready after 1 s — proceeding.\n");
      }
    }

    //last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    last_tick = HAL_GetTick();
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
		if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
			if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData)
					!= HAL_OK) {
				Error_Handler();
			}
			if (RxHeader.StdId == 0x01 && RxData[0] == 1) {
				bno055_Reset();
				bno055_Scan();
				bno055_Post();
				bno055_Init();

				  uint8_t offsets[CALIB_OFFSET_COUNT];

				  if (calib_storage_load(offsets))
				  {
				   // serialWrite("Calibration data found in Flash — restoring offsets.\n");
				    bno055_Offset_Write(offsets, CALIB_OFFSET_COUNT, BNO055_OPERATION_MODE_NDOF);
				    HAL_Delay(650);
				    //serialWrite("Offsets restored — sensor ready.\n");
				  }

				  heading_filtered = -1.0f;
				  headingOffset = 0.0f;
				  headingZeroed = false;

				  //serialWrite("Waiting for fusion engine...\n");
				    {
				      uint32_t t0 = HAL_GetTick();
				      do {
				        bno055_ReadQuatData(quat);
				        HAL_Delay(10);
				      } while (!bno055_QuatIsValid(quat) && (HAL_GetTick() - t0) < 1000U);

				      if (bno055_QuatIsValid(quat))
				      {
				        /* Seed the SLERP accumulator and heading filter */
				        quat_filtered[0] = quat[0];
				        quat_filtered[1] = quat[1];
				        quat_filtered[2] = quat[2];
				        quat_filtered[3] = quat[3];
				        heading_filtered  = bno055_QuatToHeading(quat_filtered);
				    //    serialWrite("Fusion engine ready.\n");
				      }
				      else
				      {
				      //  serialWrite("WARNING: Fusion engine not ready after 1 s — proceeding.\n");
				      }
				    }
			}
			last_tick = HAL_GetTick();
		}

        /* ---- Fixed-rate timing: block until next 10 ms slot ---------- */
        uint32_t now = HAL_GetTick();
        if ((now - last_tick) < LOOP_PERIOD_MS) {
            /* Busy-wait the remainder — keeps jitter under 1 ms */
            continue;
        }
        last_tick = now;

        /* ---- 1. Read ------------------------------------------------- */
        bno055_ReadQuatData(quat);

        /* ---- 2. Validity gate ---------------------------------------- */
        if (!bno055_QuatIsValid(quat)) {
            /* Don't print every miss at 100 Hz — rate-limit the warning */
            static uint32_t invalid_count = 0;
            invalid_count++;
            if (invalid_count % 10 == 1) {    /* print 1st, 11th, 21st... */
               // snprintf(debug, sizeof(debug),
                 //        "WARN: invalid quat x%lu\n", invalid_count);
                //serialWrite(debug);
            }
            continue;   /* skip this tick, try again next 10 ms */
        }

        /* ---- 3. SLERP ------------------------------------------------ */
        bno055_QuatSlerp(quat_filtered, quat, QUAT_SLERP_ALPHA, quat_filtered);

        /* ---- 4. Raw heading from smoothed quat ----------------------- */
        heading_raw = bno055_QuatToHeading(quat_filtered);

        /* ---- 5. Seed filter on first valid reading ------------------- */
        if (heading_filtered < 0.0f) {
            heading_filtered = heading_raw;
        }

        /* ---- 6. Jump detection --------------------------------------- */
        if (HEADING_JUMP_THRESH > 0.0f &&
            bno055_HeadingJumpDetect(heading_raw, heading_filtered, HEADING_JUMP_THRESH)) {
            static uint32_t jump_count = 0;
            jump_count++;
            //snprintf(debug, sizeof(debug),
              //       "WARN: jump #%lu  %.1f->%.1f deg\n",
                //     jump_count,
                  //   (double)heading_filtered,
                    // (double)heading_raw);
            //serialWrite(debug);
            continue;
        }

        /* ---- 7. Circular LPF ---------------------------------------- */
        heading_filtered = bno055_HeadingLPF(heading_raw, heading_filtered,
                                              HEADING_LPF_ALPHA);

		if (!headingZeroed) {
		    headingOffset = heading_filtered;
		    headingZeroed = true;
		}

		float zeroedHeading = heading_filtered - headingOffset;
		head= zeroedHeading;
		if (zeroedHeading < 0.0f) zeroedHeading += 360.0f;
		if (zeroedHeading >= 360.0f) zeroedHeading -= 360.0f;

		can_Head = (uint16_t)(zeroedHeading * 100);

        /* ---- 8. Output ---------------------------------------------- */
        canTransmit(can_Head);
        //snprintf(debug, sizeof(debug), "Heading: %.2f deg\n", (double)zeroedHeading);
        //serialWrite(debug);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void canTransmit(uint16_t val) {
	TxData[0] = (uint8_t) (val & 0xFF);
	TxData[1] = (uint8_t) ((uint16_t)val >> 8);

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}
}

void canInit_recieveMultiID(void) {
	RxHeader.IDE = CAN_ID_STD;
	RxHeader.DLC = 8;
	//rxHeader.StdId = ;
	RxHeader.RTR = CAN_RTR_DATA;

	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 1;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.FilterIdHigh = 0x1fff;
	;
	canfilterconfig.FilterIdLow = 0xffff;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0;

	if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}
}
/* USER CODE END 4 */

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
