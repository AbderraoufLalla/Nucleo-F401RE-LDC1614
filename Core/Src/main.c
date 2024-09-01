/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdlib.h>  // For strtol
#include <stdint.h>  // For uint32_t

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define the I2C address of LDC1614 (7-bit address, shift left for 8-bit format)
#define LDC1614_ADDRESS (0x2A << 1)  // Adjust based on your sensor's I2C address configuration

// Define the register to read
#define LDC1614_REG_DATA0_MSB 0x00  // Register address for DATA_MSB_CH0
#define LDC1614_REG_DATA0_LSB 0x01  // Register address for DATA_LSB_CH0
#define LDC1614_REG_DATA1_MSB 0x02  // Register address for DATA_MSB_CH1
#define LDC1614_REG_DATA1_LSB 0x03  // Register address for DATA_LSB_CH1
#define LDC1614_CH0_OFFSET 0x0C
#define LDC1614_CH1_OFFSET 0x0D
#define LDC1614_CH0_FIN_DIVIDER 0x14
#define LDC1614_CH1_FIN_DIVIDER 0x15

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
uint8_t reg_data_MSB_CH0[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_data_LSB_CH0[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_data_MSB_CH1[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_data_LSB_CH1[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_CH0_OFFSET[2];
uint8_t reg_CH1_OFFSET[2];
uint8_t reg_CH0_FIN_DIVIDER[2];
uint8_t reg_CH1_FIN_DIVIDER[2];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

// Function to read a register from LDC1614
void LDC1614_ReadRegister(uint8_t reg, uint8_t* buffer, uint16_t size) {
    HAL_I2C_Mem_Read(&hi2c1, LDC1614_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buffer, size, HAL_MAX_DELAY);
}

// Function to transmit data via UART
void Transmit_Data(uint16_t MSB_CH0, uint16_t LSB_CH0, uint16_t integerValue_MSB_CH0, uint16_t MSB_CH1, uint16_t LSB_CH1, uint16_t integerValue_MSB_CH1) {
    static uint32_t transmit_count = 0;  // Counter to keep track of transmitted data instances
    char msg[200];  // Buffer to hold the transmitted message, size increased to accommodate the count
    transmit_count++;  // Increment the counter each time data is transmitted

    // Format the data as a hexadecimal string along with the counter
    int len = snprintf(msg, sizeof(msg), "CH0 - MSB: 0x%X, LSB: 0x%X, MSB_Integer: %d | CH1 - MSB: 0x%X, LSB: 0x%X, MSB_Integer: %d. Cycle: %lu\r\n", MSB_CH0, LSB_CH0, integerValue_MSB_CH0, MSB_CH1, LSB_CH1, integerValue_MSB_CH1, transmit_count);

    // Transmit the formatted message
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
}

//// Function to combine 16-bit MSB and 12-bit LSB into a 28-bit value
//uint32_t combine_msb_lsb(uint16_t msb, uint16_t lsb) {
//    // Mask the LSB to ensure it is only 12 bits
//    uint16_t lsb_12bit = lsb & 0x0FFF;
//    // Shift the MSB 12 bits to the left and combine with the LSB
//    uint32_t combined_value = ((uint32_t)msb << 12) | lsb_12bit;
//    return combined_value;
//}
//// Function to combine 16-bit MSB and 12-bit LSB into a 28-bit decimal value
//uint32_t combine_msb_lsb_to_decimal(uint16_t msb, uint16_t lsb) {
//    // Mask the LSB to ensure only the lower 12 bits are used
//    uint16_t lsb_12bit = lsb & 0x0FFF;
//    // Combine MSB and LSB to form a 28-bit value
//    uint32_t combined_value = ((uint32_t)msb << 12) | lsb_12bit;
//    return combined_value;
//}
//uint32_t combine_msb_lsb_to_integer(uint16_t msb, uint16_t lsb) {
//    // Mask the LSB to ensure only the lower 12 bits are used
//    uint16_t lsb_12bit = lsb & 0x0FFF;
//    // Combine MSB and LSB to form a 28-bit value
//    uint32_t combined_value = ((uint32_t)msb << 12) | lsb_12bit;
//    return combined_value;
//}
//// Function to convert the combined 28-bit value to a float
//float combine_msb_lsb_to_float(uint16_t msb, uint16_t lsb) {
//    uint32_t integer_value = combine_msb_lsb_to_integer(msb, lsb);
//    return (float)integer_value;
//}
//int calculate_bits(uint32_t value) {
//    int bits = 0;
//    while (value > 0) {
//        bits++;
//        value >>= 1; // Right shift by 1 bit
//    }
//    return bits;
//}

int hex_to_dec(uint16_t hex) {
	char hexString_CH0[5];
	snprintf(hexString_CH0, sizeof(hexString_CH0), "%04X", hex);
	int integerValue = (int)strtol(hexString_CH0, NULL, 16);
	return integerValue;
}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    // Read 2 bytes from register 0x00 of LDC1614
    LDC1614_ReadRegister(LDC1614_REG_DATA0_MSB, reg_data_MSB_CH0, 2);
    LDC1614_ReadRegister(LDC1614_REG_DATA0_LSB, reg_data_LSB_CH0, 2);
    LDC1614_ReadRegister(LDC1614_REG_DATA1_MSB, reg_data_MSB_CH1, 2);
    LDC1614_ReadRegister(LDC1614_REG_DATA1_LSB, reg_data_LSB_CH1, 2);
    LDC1614_ReadRegister(LDC1614_CH0_OFFSET, reg_CH0_OFFSET, 2);
    LDC1614_ReadRegister(LDC1614_CH1_OFFSET, reg_CH1_OFFSET, 2);
    LDC1614_ReadRegister(LDC1614_CH0_FIN_DIVIDER, reg_CH0_FIN_DIVIDER, 2);
    LDC1614_ReadRegister(LDC1614_CH1_FIN_DIVIDER, reg_CH1_FIN_DIVIDER, 2);


    // Combine the two bytes into a single 16-bit value
    uint16_t MSB_CH0 = (reg_data_MSB_CH0[0] << 8) | reg_data_MSB_CH0[1];
    uint16_t LSB_CH0= (reg_data_LSB_CH0[0] << 8) | reg_data_LSB_CH0[1];
    uint16_t MSB_CH1 = (reg_data_MSB_CH1[0] << 8) | reg_data_MSB_CH1[1];
    uint16_t LSB_CH1= (reg_data_LSB_CH1[0] << 8) | reg_data_LSB_CH1[1];

	// Convert the 16-bit value to a hexadecimal string
	char hexString_CH0[5]; // 4 digits + null terminator
	snprintf(hexString_CH0, sizeof(hexString_CH0), "%04X", MSB_CH0);
	char hexString_CH1[5]; // 4 digits + null terminator
	snprintf(hexString_CH1, sizeof(hexString_CH1), "%04X", MSB_CH1);

    // Convert the hexadecimal string to an integer
    int integerValue_MSB_CH0 = hex_to_dec(MSB_CH0);
    int integerValue_MSB_CH1 = (int)strtol(hexString_CH1, NULL, 16);
    //unsigned long fullHexNumber = ((unsigned long)MSB_CH0 << 12) | (LSB_CH0 & 0xFFF);

    // int DATA0 = integerValue_MSB_CH0 * 4096 +
    // Transmit the register value via UART
    Transmit_Data(MSB_CH0, LSB_CH0, integerValue_MSB_CH0, MSB_CH1, LSB_CH1, integerValue_MSB_CH1);

    if (integerValue_MSB_CH0 > 310 || integerValue_MSB_CH1 > 310 ){
    	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }else{
    	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    };
    // Add a delay or condition to control the transmission frequency
    HAL_Delay(100);  // Delay for 1 second, adjust as needed

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
