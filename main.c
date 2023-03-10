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
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "string.h"

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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t imu_id;
int8_t channel1[100];
int8_t channel2[100];
int count;
uint8_t check_offset = 0;
uint8_t checkbyte;
int8_t  SplByte_angle_x[2];
int8_t  SplByte_angle_y[2];
int8_t SplByte[2];
uint8_t SplByteCycle[2];
uint8_t int_status;
uint32_t time_get_gyro, time_get_gyro_tmp;
uint32_t previousMicros;
uint32_t Looptimer;
uint16_t currentMicros;

// gyro variables
int16_t gyro_value_x;
int16_t gyro_value_y;
int16_t gyro_value_z;
int32_t gyro_x_total;
int32_t gyro_y_total;
int32_t gyro_z_total;
int16_t gyro_offset_x;
int16_t gyro_offset_y;
int16_t gyro_offset_z;
float gyro_angle_x;
float gyro_angle_y;
float gyro_angle_z;
float angle_x,angle_y,angle_z;
// accel variables
int16_t accel_value_x;
int16_t accel_value_y;
int16_t accel_value_z;
int32_t acc_x_total;
int32_t acc_y_total;
int32_t acc_z_total;
int16_t acc_offset_x;
int16_t acc_offset_y;
int16_t acc_offset_z;
float acc_vector_total, referAcc_vector,accel_Xtranfer,accel_Ytranfer,accel_Ztranfer;
uint16_t vibra;
int16_t angle_x_10;
int16_t angle_y_10;
float acc_angle_x;
float acc_angle_y;
float acc_angle_z;

// temperature variable
float temp_value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
// void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(hspi);
//  if(hspi -> Instance == SPI2)
//  {
//	  HAL_SPI_Receive_IT(&hspi2,&rx_buffer,1);
//  }
//}
// retarget hàm fput để truyền dữ liệu qua serial bằng printf
#if defined(__GNUC__)
int _write(int fd, char * ptr, int len) {
  HAL_UART_Transmit( & huart1, (uint8_t * ) ptr, len, HAL_MAX_DELAY);
  return len;
}
#elif defined(__ICCARM__)#include "LowLevelIOInterface.h"

size_t __write(int handle,
  const unsigned char * buffer, size_t size) {
  HAL_UART_Transmit( & huart1, (uint8_t * ) buffer, size, HAL_MAX_DELAY);
  return size;
}
#elif defined(__CC_ARM)
int fputc(int ch, FILE * f) {
  HAL_UART_Transmit( & huart1, (uint8_t * ) & ch, 1, HAL_MAX_DELAY);
  return ch;
}
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 uint8_t readReg(uint8_t reg)
 {
	 uint8_t sendByte, receiveByte;

	 sendByte = 0x80 | reg;

	 /// select
	 HAL_GPIO_WritePin(SPI2_NSS1_GPIO_Port, SPI2_NSS1_Pin, GPIO_PIN_RESET);

	 /// transmit
	 HAL_SPI_Transmit(&hspi2, &sendByte, 1, 100);

	 /// receive
	 HAL_SPI_Receive(&hspi2, &receiveByte, 1, 100);

	 /// unuselect
	 HAL_GPIO_WritePin(SPI2_NSS1_GPIO_Port, SPI2_NSS1_Pin, GPIO_PIN_SET);

	 ///
	 return receiveByte;

 }
 int16_t getData(uint8_t reg1, uint8_t reg2)
 {
  uint16_t data;
  data = ((int16_t)readReg(reg1) << 8) | ((int16_t)readReg(reg2));
  return data;
 }
 //
 void get_imu_data()
 {
	  // temperature data
	  temp_value =(float)getData(0x1D,0x1E);
	  // Temperature in C
	  temp_value = (temp_value/132.48f) + 25.0f;
	  // gyro data

	  accel_value_x = getData(0x1F, 0x20);
	  accel_value_y = getData(0x21, 0x22);
	  accel_value_z = getData(0x23, 0x24);
	  // transfer accel's unit from digit to m/s/s
	  accel_Xtranfer = (float)((float)accel_value_x*9.8f/8192.0f);
	  accel_Ytranfer = (float)((float)accel_value_y*9.8f/8192.0f);
	  accel_Ztranfer = (float)((float)accel_value_z*9.8f/8192.0f);
	  // accel data

	  gyro_value_x = getData(0x25, 0x26);
	  gyro_value_y = getData(0x27, 0x28);
	  gyro_value_z = getData(0x29, 0x2A);
	  if(check_offset == 1)
	  {
		  //gyro
		  gyro_value_x -= gyro_offset_x;
		  gyro_value_y -= gyro_offset_y;
		  gyro_value_z -= gyro_offset_z;
		  //accel
		  accel_value_x -= acc_offset_x;
		  accel_value_y -= acc_offset_y;
	  }
 }

 // finding offset to increase the raw data gyro's accuracy
 void get_offset()
 {

	 // get medium of 2000 raw data
	 for(count = 0; count < 2000; count++)
	 {
		 get_imu_data();
       //raw gyro data
		 gyro_x_total += gyro_value_x;
		 gyro_y_total += gyro_value_y;
		 gyro_z_total += gyro_value_z;
	   // raw accel data
		 acc_x_total += accel_value_x;
		 acc_y_total += accel_value_y;
		 acc_z_total += accel_value_z;
	 }
	 if(count==2000)
	 {
		//gyro offset
		gyro_offset_x = gyro_x_total/2000;
		gyro_offset_y = gyro_y_total/2000;
		gyro_offset_z = gyro_z_total/2000;
		//accel offset
		acc_offset_x = acc_x_total/2000;
		acc_offset_y = acc_y_total/2000;
	//	acc_offset_z = acc_z_total/2000;
		check_offset = 1;
		count = 0;
	 }
 }
 void writeReg(uint8_t reg, uint8_t data)
 {
	 uint8_t sendByte[2];

	 sendByte[0] = reg;
	 sendByte[1] = data;

	 /// select
	 HAL_GPIO_WritePin(SPI2_NSS1_GPIO_Port, SPI2_NSS1_Pin, GPIO_PIN_RESET);

	 /// transmit
	 HAL_SPI_Transmit(&hspi2, sendByte, 2, 100);

	 /// unuselect
	 HAL_GPIO_WritePin(SPI2_NSS1_GPIO_Port, SPI2_NSS1_Pin, GPIO_PIN_SET);
 }
 void refer_acc_vector()
 {
 	 for(count = 0;count<50;count++)
	 {
 		 acc_vector_total = sqrt((accel_value_x*accel_value_x)+(accel_value_y*accel_value_y)+(accel_value_z*accel_value_z));
 		 referAcc_vector += acc_vector_total;
	 }
 	 if(count==50)
 		{
 		 referAcc_vector = referAcc_vector/50;
 		}
 }
 void setup_imu()
 {
	  writeReg(0x11, 0x01); // soft reset
	  HAL_Delay(10);
	  // do not issue any register writes for 200µs
	  // HAL_Delay(2);
      writeReg(0x50, 0x46); // accel config: +-4g, 1khz
	  HAL_Delay(10);
	  writeReg(0x4F, 0x46); // gyro config : +-500dps, 1khz
	  HAL_Delay(10);
	  writeReg(0x4E, 0x0F); // enable power
	  HAL_Delay(10);
 }
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Start timebase
  HAL_TIM_Base_Start(&htim2);

 // writeReg(0x11, 0x01); // soft reset
  setup_imu();
  HAL_Delay(100);
  get_offset();
  HAL_Delay(100);
  refer_acc_vector();
  HAL_Delay(100);
  Looptimer = TIM2->CNT;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  checkbyte = readReg(0x75); // check who am i
	  imu_id = readReg(0x50);
      /// Kiem tra co data ready
      int_status = readReg(0x2D);

      /// Xu ly khi co du lieu moi
 /*    if(int_status & 0x08)
      {
    	  /// Do du lieu
          get_imu_data();

          /// Tinh thoi gian nhan gyro
          time_get_gyro 	= TIM2->CNT - time_get_gyro_tmp;
    	  time_get_gyro_tmp = TIM2->CNT;
  */

      /// Calculate angle by gyro
      get_imu_data();
      //0.00006106 = 1/(65.5*250);   with 50hz = 1/(4ms)
      angle_x += gyro_value_y*0.00006106f;
      angle_y += gyro_value_x*0.00006106f;

      ///Calculate angle by accel
      acc_vector_total = sqrt((accel_value_x*accel_value_x)+(accel_value_y*accel_value_y)+(accel_value_z*accel_value_z));
      //57.296 = 180/3.14----radian to degree
      acc_angle_x = asin(-1*accel_value_x/acc_vector_total)*57.296;
      acc_angle_y = asin(accel_value_y/acc_vector_total)*57.296;
      // using complementary to calib angle
      angle_x = angle_x*0.96f + acc_angle_x*0.04f;
      angle_y = angle_y*0.96f + acc_angle_y*0.04f;

   //   sprintf(channel1,"%d\n",gyro_value_x);
      // x10 angle
      angle_x_10 = angle_x*10;
      angle_y_10 = angle_y*10;

      // tách 2byte từ kiểu in16 thành 2 byte kiểu int8 để truyền uart được chính xác
      SplByte_angle_x[0] = (int8_t)(angle_x_10 >> 8);
      SplByte_angle_x[1] = (int8_t)(angle_x_10 & 0x00FF);
      SplByte_angle_y[0] = (int8_t)(angle_y_10 >> 8);
      SplByte_angle_y[1] = (int8_t)(angle_y_10 & 0x00FF);

      //======= Plot angle data
//       HAL_UART_Transmit(&huart1,&SplByte_angle_x,2, 100);
//       HAL_UART_Transmit(&huart1,&SplByte_angle_y,2, 100);
	  // HAL_Delay(100);
      // vibration
      vibra = (uint16_t)(acc_vector_total - referAcc_vector);
      SplByte[0] = (int8_t)(vibra >> 8);
      SplByte[1] = (int8_t)(vibra & 0x00FF);
      currentMicros ++;
      SplByteCycle[0] = (int8_t)(currentMicros >> 8);
      SplByteCycle[1] = (int8_t)(currentMicros & 0x00FF);
     //currentMicros = TIM2->CNT - Looptimer;
      while(TIM2->CNT - Looptimer < 4000);
      ; // check loop
      HAL_UART_Transmit(&huart1,&SplByte,2, 100);
      HAL_UART_Transmit(&huart1,&SplByteCycle,2, 100);
      Looptimer = TIM2->CNT;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_NSS2_Pin|SPI2_NSS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI2_NSS2_Pin SPI2_NSS1_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS2_Pin|SPI2_NSS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
