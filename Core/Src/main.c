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
#include <stdio.h>
#include <stdlib.h>  // For strtol
#include <stdint.h>  // For uint32_t
#include <string.h>
#include <math.h>
#include <ctype.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define the I2C address of LDC1614 (7-bit address, shift left for 8-bit format)
#define LDC1614_ADDRESS (0x2A << 1)  // Adjust based on your sensor's I2C address configuration

// Define Channels data registers
#define LDC1614_REG_DATA0_MSB 0x00  // Register address for DATA_MSB_CH0 // Read Only
#define LDC1614_REG_DATA0_LSB 0x01  // Register address for DATA_LSB_CH0 // Read Only
#define LDC1614_REG_DATA1_MSB 0x02  // Register address for DATA_MSB_CH1 // Read Only
#define LDC1614_REG_DATA1_LSB 0x03  // Register address for DATA_LSB_CH1 // Read Only
#define LDC1614_REG_DATA2_MSB 0x04  // Register address for DATA_MSB_CH2 // Read Only
#define LDC1614_REG_DATA2_LSB 0x05  // Register address for DATA_LSB_CH2 // Read Only
#define LDC1614_REG_DATA3_MSB 0x06  // Register address for DATA_MSB_CH3 // Read Only
#define LDC1614_REG_DATA3_LSB 0x07  // Register address for DATA_LSB_CH3 // Read Only

// Define Channels offsets registers
#define LDC1614_CH0_OFFSET 0x0C // Offset channel 0 // Read & Write
#define LDC1614_CH1_OFFSET 0x0D // Offset channel 1 // Read & Write
#define LDC1614_CH2_OFFSET 0x0E // Offset channel 2 // Read & Write
#define LDC1614_CH3_OFFSET 0x0F // Offset channel 3 // Read & Write

// Define Channels dividers registers
#define LDC1614_CH0_FIN_DIVIDER 0x14 // Divider channel 0 // Read & Write
#define LDC1614_CH1_FIN_DIVIDER 0x15 // Divider channel 1 // Read & Write
#define LDC1614_CH2_FIN_DIVIDER 0x16 // Divider channel 2 // Read & Write
#define LDC1614_CH3_FIN_DIVIDER 0x17 // Divider channel 3 // Read & Write

// Define RCOUNT registers
#define RCOUNT_CH0 0x08
#define RCOUNT_CH1 0x09
#define RCOUNT_CH2 0x0A
#define RCOUNT_CH3 0x0B

// Define SETTLECOUNT registers
#define SETTLECOUNT_CH0 0x10
#define SETTLECOUNT_CH1 0x11
#define SETTLECOUNT_CH2 0x12
#define SETTLECOUNT_CH3 0x13


// Define MUX configuration register
#define CONFIG_reg 0x1A
#define MUX_reg 0x1B
#define RESET_DEV 0x1C


#define DRIVE_CURRENT_CH0 0x1E
#define DRIVE_CURRENT_CH1 0x1F
#define DRIVE_CURRENT_CH2 0x20
#define DRIVE_CURRENT_CH3 0x21

// THIS IS YOUR FIRST BRANCH

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

// Buffers for channels data
uint8_t reg_data_MSB_CH0[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_data_LSB_CH0[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_data_MSB_CH1[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_data_LSB_CH1[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_data_MSB_CH2[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_data_LSB_CH2[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_data_MSB_CH3[2];  // Buffer to hold the read data (2 bytes for LDC1614)
uint8_t reg_data_LSB_CH3[2];  // Buffer to hold the read data (2 bytes for LDC1614)

// Buffers for channels offsets
uint8_t reg_CH0_OFFSET[2];
uint8_t reg_CH1_OFFSET[2];
uint8_t reg_CH2_OFFSET[2];
uint8_t reg_CH3_OFFSET[2];

// Buffers for channels frequency dividers
uint8_t reg_CH0_FIN_DIVIDER[2];
uint8_t reg_CH1_FIN_DIVIDER[2];
uint8_t reg_CH2_FIN_DIVIDER[2];
uint8_t reg_CH3_FIN_DIVIDER[2];

//MUX Configurations (2, 3 or 4 channels)
uint8_t config_channels_DeGlitch[2] = {0x82, 0x0C};  // Configuration to be written, change here.

// Configurations (2, 3 or 4 channels)
uint8_t config_2Channels_1MHz[2] = {0x82, 0x09};  // Configuration to be written for 2 channels mode // Deglitsh 1MHz
uint8_t config_3Channels_1MHz[2] = {0xA2, 0x09};  // Configuration to be written for 3 channels mode // Deglitsh 1MHz
uint8_t config_4Channels_1MHz[2] = {0xC2, 0x09};  // Configuration to be written for 4 channels mode // Deglitsh 1MHz

uint8_t config_2Channels_3MHz[2] = {0x82, 0x0C};  // Configuration to be written for 2 channels mode // Deglitsh 3.3MHz
uint8_t config_3Channels_3MHz[2] = {0xA2, 0x0C};  // Configuration to be written for 3 channels mode // Deglitsh 3.3MHz
uint8_t config_4Channels_3MHz[2] = {0xC2, 0x0C};  // Configuration to be written for 4 channels mode // Deglitsh 3.3MHz

uint8_t config_2Channels_10MHz[2] = {0x82, 0x0D};  // Configuration to be written for 2 channels mode // Deglitsh 10MHz
uint8_t config_3Channels_10MHz[2] = {0xA2, 0x0D};  // Configuration to be written for 3 channels mode // Deglitsh 10MHz
uint8_t config_4Channels_10MHz[2] = {0xC2, 0x0D};  // Configuration to be written for 4 channels mode // Deglitsh 10MHz

uint8_t config_2Channels_33MHz[2] = {0x82, 0x0F};  // Configuration to be written for 2 channels mode // Deglitsh 33MHz
uint8_t config_3Channels_33MHz[2] = {0xA2, 0x0F};  // Configuration to be written for 3 channels mode // Deglitsh 33MHz
uint8_t config_4Channels_33MHz[2] = {0xC2, 0x0F};  // Configuration to be written for 4 channels mode // Deglitsh 33MHz

// COUNTS SETTINGS
uint8_t R_COUNT_DEFAULT[2] = {0xFF, 0xFF};
uint8_t SETTLECOUNT_DEFAULT[2] = {0x04, 0x00};

// CLOCK SETTINGS
uint8_t clk_src[2] = {0x1c, 0x01};
uint8_t reset_value[2] = {0x80, 0x00};
uint8_t active_value[2] = {0x00, 0x00};

uint8_t DIVIDER_DEFAULT[2] = {0x10, 0x0E};

uint8_t DRIVE_CURRENT_DEFAULT[2] = {0x8C, 0x40};


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

void LDC1614_WriteRegister(uint8_t reg, uint8_t* data, uint16_t size) {
    HAL_I2C_Mem_Write(&hi2c1, LDC1614_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

// Function to transform hex to dec
int hex_to_dec(uint16_t hex) {
	char hexString_CH0[5];
	snprintf(hexString_CH0, sizeof(hexString_CH0), "%04X", hex);
	int integerValue = (int)strtol(hexString_CH0, NULL, 16);
	return integerValue;
}

void LDC1614_WriteRegister_LSB(uint16_t reg, uint16_t* data, uint16_t size) {
    uint8_t current_data[2];  // Array to hold the current register value (MSB + LSB)

    // Read the current value of the register (assuming the register is 16 bits)
    HAL_I2C_Mem_Read(&hi2c1, LDC1614_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, current_data, 2, HAL_MAX_DELAY);

    // Update the LSB (first 8 bits) while keeping the MSB (second 8 bits) unchanged
    current_data[1] = data;  // Write new LSB from the input data

    // Write the updated register value (MSB unchanged, LSB updated)
    HAL_I2C_Mem_Write(&hi2c1, LDC1614_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, current_data, 2, HAL_MAX_DELAY);
}

// Function to transmit data via UART
void Transmit_Data(uint16_t MSB_CH0, uint16_t LSB_CH0, uint16_t CH0_FIN_DIVIDER, uint16_t CH0_OFFSET, uint16_t MSB_CH1, uint16_t LSB_CH1, uint16_t CH1_FIN_DIVIDER, uint16_t CH1_OFFSET, uint16_t MSB_CH2, uint16_t LSB_CH2, uint16_t CH2_FIN_DIVIDER, uint16_t CH2_OFFSET, uint16_t MSB_CH3, uint16_t LSB_CH3, uint16_t CH3_FIN_DIVIDER, uint16_t CH3_OFFSET) {
    static uint32_t transmit_count = 0;  // Counter to keep track of transmitted data instances
    char msg[1000];  // Buffer to hold the transmitted message, size increased to accommodate the count
    transmit_count++;  // Increment the counter each time data is transmitted

    uint16_t MSB_CH0_masked = MSB_CH0 & 0x0FFF; // because LSB is only 12 bits
    uint16_t MSB_CH1_masked = MSB_CH1 & 0x0FFF; // because LSB is only 12 bits
    uint16_t MSB_CH2_masked = MSB_CH2 & 0xFFF; // because LSB is only 12 bits
    uint16_t MSB_CH3_masked = MSB_CH3 & 0xFFF; // because LSB is only 12 bits



    uint16_t CH0_FIN_DIVIDER_masked = CH0_FIN_DIVIDER & 0x00FF; // Only the first 8 bits represents the FIN divider.
    uint16_t CH1_FIN_DIVIDER_masked = CH1_FIN_DIVIDER & 0x00FF; // Only the first 8 bits represents the FIN divider.
    uint16_t CH2_FIN_DIVIDER_masked = CH2_FIN_DIVIDER & 0x00FF; // Only the first 8 bits represents the FIN divider.
    uint16_t CH3_FIN_DIVIDER_masked = CH3_FIN_DIVIDER & 0x00FF; // Only the first 8 bits represents the FIN divider.

    // Format the data as a hexadecimal string along with the counter
    int len = snprintf(msg, sizeof(msg), "CH0 - MSB: %d, LSB: %d, F_DIV: %d, OFFSET: %d | CH1 - MSB: %d, LSB: %d, F_DIV: %d, OFFSET: %d | CH2 - MSB: %d, LSB: %d, F_DIV: %d, OFFSET: %d | CH3 - MSB: %d, LSB: %d, F_DIV: %d, OFFSET: %d - Cycle: %d \r\n", MSB_CH0_masked, LSB_CH0, CH0_FIN_DIVIDER_masked, CH0_OFFSET, MSB_CH1_masked, LSB_CH1, CH1_FIN_DIVIDER_masked, CH1_OFFSET, MSB_CH2_masked, LSB_CH2, CH2_FIN_DIVIDER_masked, CH2_OFFSET, MSB_CH3_masked, LSB_CH3, CH3_FIN_DIVIDER_masked, CH3_OFFSET, transmit_count);

    // Transmit the formatted message
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
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
  LDC1614_WriteRegister(RESET_DEV, reset_value, 2);
  LDC1614_WriteRegister(RESET_DEV, active_value, 2);


  // Initialising RCOUNT
  LDC1614_WriteRegister(RCOUNT_CH0, R_COUNT_DEFAULT, 2);
  LDC1614_WriteRegister(RCOUNT_CH1, R_COUNT_DEFAULT, 2);
  LDC1614_WriteRegister(RCOUNT_CH2, R_COUNT_DEFAULT, 2);
  LDC1614_WriteRegister(RCOUNT_CH3, R_COUNT_DEFAULT, 2);

  // Initialising SETTLECOUNT
  LDC1614_WriteRegister(SETTLECOUNT_CH0, SETTLECOUNT_DEFAULT, 2);
  LDC1614_WriteRegister(SETTLECOUNT_CH1, SETTLECOUNT_DEFAULT, 2);
  LDC1614_WriteRegister(SETTLECOUNT_CH2, SETTLECOUNT_DEFAULT, 2);
  LDC1614_WriteRegister(SETTLECOUNT_CH3, SETTLECOUNT_DEFAULT, 2);

  //Frequency divider configuration for the LDC1614
  LDC1614_WriteRegister(LDC1614_CH0_FIN_DIVIDER, DIVIDER_DEFAULT, 2); // Setting the frequency divider as 12 // F_IN = 1 // Equal to resonance frequency + 10%
  LDC1614_WriteRegister(LDC1614_CH1_FIN_DIVIDER, DIVIDER_DEFAULT, 2); // Setting the frequency divider as 12 // F_IN = 1 // Equal to resonance frequency + 10%
  LDC1614_WriteRegister(LDC1614_CH2_FIN_DIVIDER, DIVIDER_DEFAULT, 2); // Setting the frequency divider as 12 // F_IN = 1 // Equal to resonance frequency + 10%
  LDC1614_WriteRegister(LDC1614_CH3_FIN_DIVIDER, DIVIDER_DEFAULT, 2); // Setting the frequency divider as 12 // F_IN = 1 // Equal to resonance frequency + 10%

  //Clock configuration: Internal clock
  LDC1614_WriteRegister(CONFIG_reg, clk_src, 2);

  //Number of channels configuration for the LDC1614 & Deglitch
  LDC1614_WriteRegister(MUX_reg, config_2Channels_3MHz, 2);

  LDC1614_WriteRegister(DRIVE_CURRENT_CH0, DRIVE_CURRENT_DEFAULT, 2);
  LDC1614_WriteRegister(DRIVE_CURRENT_CH1, DRIVE_CURRENT_DEFAULT, 2);
  LDC1614_WriteRegister(DRIVE_CURRENT_CH2, DRIVE_CURRENT_DEFAULT, 2);
  LDC1614_WriteRegister(DRIVE_CURRENT_CH3, DRIVE_CURRENT_DEFAULT, 2);


  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    // Read data
    LDC1614_ReadRegister(LDC1614_REG_DATA0_MSB, reg_data_MSB_CH0, 2);
    LDC1614_ReadRegister(LDC1614_REG_DATA0_LSB, reg_data_LSB_CH0, 2);
    LDC1614_ReadRegister(LDC1614_REG_DATA1_MSB, reg_data_MSB_CH1, 2);
    LDC1614_ReadRegister(LDC1614_REG_DATA1_LSB, reg_data_LSB_CH1, 2);
    LDC1614_ReadRegister(LDC1614_REG_DATA2_MSB, reg_data_MSB_CH2, 2);
    LDC1614_ReadRegister(LDC1614_REG_DATA2_LSB, reg_data_LSB_CH2, 2);
    LDC1614_ReadRegister(LDC1614_REG_DATA3_MSB, reg_data_MSB_CH3, 2);
    LDC1614_ReadRegister(LDC1614_REG_DATA3_LSB, reg_data_LSB_CH3, 2);


    LDC1614_ReadRegister(LDC1614_CH0_OFFSET, reg_CH0_OFFSET, 2);
    LDC1614_ReadRegister(LDC1614_CH1_OFFSET, reg_CH1_OFFSET, 2);
    LDC1614_ReadRegister(LDC1614_CH2_OFFSET, reg_CH2_OFFSET, 2);
    LDC1614_ReadRegister(LDC1614_CH3_OFFSET, reg_CH3_OFFSET, 2);

    LDC1614_ReadRegister(LDC1614_CH0_FIN_DIVIDER, reg_CH0_FIN_DIVIDER, 2);
    LDC1614_ReadRegister(LDC1614_CH1_FIN_DIVIDER, reg_CH1_FIN_DIVIDER, 2);
    LDC1614_ReadRegister(LDC1614_CH2_FIN_DIVIDER, reg_CH2_FIN_DIVIDER, 2);
    LDC1614_ReadRegister(LDC1614_CH3_FIN_DIVIDER, reg_CH3_FIN_DIVIDER, 2);

    // Combine the two bytes into a single 16-bit value
    uint16_t MSB_CH0 = (reg_data_MSB_CH0[0] << 8) | reg_data_MSB_CH0[1];
    uint16_t LSB_CH0= (reg_data_LSB_CH0[0] << 8) | reg_data_LSB_CH0[1];
    uint16_t MSB_CH1 = (reg_data_MSB_CH1[0] << 8) | reg_data_MSB_CH1[1];
    uint16_t LSB_CH1= (reg_data_LSB_CH1[0] << 8) | reg_data_LSB_CH1[1];
    uint16_t MSB_CH2 = (reg_data_MSB_CH2[0] << 8) | reg_data_MSB_CH2[1];
    uint16_t LSB_CH2= (reg_data_LSB_CH2[0] << 8) | reg_data_LSB_CH2[1];
    uint16_t MSB_CH3 = (reg_data_MSB_CH3[0] << 8) | reg_data_MSB_CH3[1];
    uint16_t LSB_CH3= (reg_data_LSB_CH3[0] << 8) | reg_data_LSB_CH3[1];

    uint16_t CH0_OFFSET= (reg_CH0_OFFSET[0] << 8) | reg_CH0_OFFSET[1];
    uint16_t CH1_OFFSET= (reg_CH1_OFFSET[0] << 8) | reg_CH1_OFFSET[1];
    uint16_t CH2_OFFSET= (reg_CH2_OFFSET[0] << 8) | reg_CH2_OFFSET[1];
    uint16_t CH3_OFFSET= (reg_CH3_OFFSET[0] << 8) | reg_CH3_OFFSET[1];

    uint16_t CH0_FIN_DIVIDER= (reg_CH0_FIN_DIVIDER[0] << 8) | reg_CH0_FIN_DIVIDER[1];
    uint16_t CH1_FIN_DIVIDER= (reg_CH1_FIN_DIVIDER[0] << 8) | reg_CH1_FIN_DIVIDER[1];
    uint16_t CH2_FIN_DIVIDER= (reg_CH2_FIN_DIVIDER[0] << 8) | reg_CH2_FIN_DIVIDER[1];
    uint16_t CH3_FIN_DIVIDER= (reg_CH3_FIN_DIVIDER[0] << 8) | reg_CH3_FIN_DIVIDER[1];


    // Data transform for output LED
    int integerValue_MSB_CH0 = hex_to_dec(MSB_CH0);
    int integerValue_MSB_CH1 = hex_to_dec(MSB_CH1);


    // Transmit the register value via UART
    Transmit_Data(MSB_CH0, LSB_CH0, CH0_FIN_DIVIDER, CH0_OFFSET, MSB_CH1, LSB_CH1, CH1_FIN_DIVIDER, CH1_OFFSET, MSB_CH2, LSB_CH2, CH2_FIN_DIVIDER, CH2_OFFSET, MSB_CH3, LSB_CH3, CH3_FIN_DIVIDER, CH3_OFFSET);

    if (integerValue_MSB_CH0 > 3700 || integerValue_MSB_CH1 > 3700 ){
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
